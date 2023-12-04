import math
import os
from typing import Any, TextIO
from xml.etree import ElementTree as ET

import numpy as np
import numpy.typing as npt

from . import stl_combine


def rotation_matrix_to_euler_angles(
    r: npt.NDArray[np.float64],
) -> npt.NDArray[np.float64]:
    sy = math.sqrt(r[0, 0] * r[0, 0] + r[1, 0] * r[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(r[2, 1], r[2, 2])
        y = math.atan2(-r[2, 0], sy)
        z = math.atan2(r[1, 0], r[0, 0])
    else:
        x = math.atan2(-r[1, 2], r[1, 1])
        y = math.atan2(-r[2, 0], sy)
        z = 0

    return np.array([x, y, z])


def add_origin_element(parent: ET.Element, matrix: npt.NDArray[np.float64]) -> None:
    # TODO: Make this not use '%' for formatting
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]
    xyz_str = "%.20g %.20g %.20g" % (x, y, z)

    r, p, y = rotation_matrix_to_euler_angles(matrix)
    rpy_str = "%.20g %.20g %.20g" % (r, p, y)

    ET.SubElement(parent, "origin", xyz=xyz_str, rpy=rpy_str)


class RobotDescription:
    def __init__(self, name: str):
        self.draw_collisions = False
        self.relative = True
        self.merge_stls = "no"
        self.merge_stls_collisions = False
        self.use_fixed_links = False
        self.simplify_stls = "no"
        self.max_stl_size = 3
        self.joint_max_effort = 1
        self.joint_max_velocity = 10
        self.no_dynamics = False
        self.package_name = ""
        self.add_dummy_base_link = False
        self.robot_name = name
        self.mesh_dir = None

        self._mesh = {"visual": None, "collision": None}
        self._color = np.array([0.0, 0.0, 0.0])
        self._color_mass = 0.0
        self._link_childs = 0
        self._visuals: list[list[Any]] = []
        self._dynamics: list[dict[str, Any]] = []

    def should_merge_stls(self, node: str) -> bool:
        return self.merge_stls == "all" or self.merge_stls == node

    def should_simplify_stls(self, node: str) -> bool:
        return self.simplify_stls == "all" or self.simplify_stls == node

    def joint_max_effort_for(self, joint_name: str) -> float:
        if isinstance(self.joint_max_effort, dict):
            if joint_name in self.joint_max_effort:
                return self.joint_max_effort[joint_name]
            else:
                return self.joint_max_effort["default"]
        else:
            return self.joint_max_effort

    def joint_max_velocity_for(self, joint_name: str) -> float:
        if isinstance(self.joint_max_velocity, dict):
            if joint_name in self.joint_max_velocity:
                return self.joint_max_velocity[joint_name]
            else:
                return self.joint_max_velocity["default"]
        else:
            return self.joint_max_velocity

    def reset_link(self) -> None:
        self._mesh = {"visual": None, "collision": None}
        self._color = np.array([0.0, 0.0, 0.0])
        self._color_mass = 0
        self._link_childs = 0
        self._visuals = []
        self._dynamics = []

    def add_link_dynamics(
        self, matrix: npt.NDArray[np.float64], mass: float, com: npt.NDArray[np.float64], inertia: npt.NDArray[np.float64]
    ) -> None:
        # Inertia
        I = np.matrix(np.reshape(inertia[:9], (3, 3)))
        R = matrix[:3, :3]
        # Expressing COM in the link frame
        com = np.array((matrix * np.matrix([com[0], com[1], com[2], 1]).T).T)[0][:3]
        # Expressing inertia in the link frame
        inertia = R * I * R.T

        self._dynamics.append({"mass": mass, "com": com, "inertia": inertia})

    def merge_stl(
        self,
        stl: str,
        matrix: npt.NDArray[np.float64],
        color: npt.NDArray[np.float64],
        mass: float,
        node: str = "visual",
    ) -> None:
        if node == "visual":
            self._color += np.array(color) * mass
            self._color_mass += mass

        m = stl_combine.load_mesh(stl)
        stl_combine.apply_matrix(m, matrix)

        if self._mesh[node] is None:
            self._mesh[node] = m
        else:
            self._mesh[node] = stl_combine.combine_meshes(self._mesh[node], m)

    def link_dynamics(
        self,
    ) -> tuple[float, npt.NDArray[np.float64], npt.NDArray[np.float64]]:
        mass = 0
        com = np.array([0.0] * 3)
        inertia = np.matrix(np.zeros((3, 3)))
        identity = np.matrix(np.eye(3))

        for dynamic in self._dynamics:
            mass += dynamic["mass"]
            com += dynamic["com"] * dynamic["mass"]

        if mass > 0:
            com /= mass

        # https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=246
        for dynamic in self._dynamics:
            r = dynamic["com"] - com
            p = np.matrix(r)
            inertia += (  # type: ignore[misc]
                dynamic["inertia"]
                + (np.dot(r, r) * identity - p.T * p) * dynamic["mass"]
            )

        return mass, com, inertia


class RobotURDF(RobotDescription):
    def __init__(self, name: str):
        super().__init__(name)
        self.ext = "urdf"
        self.xml_root = ET.Element("robot", name=self.robot_name)
        self._active_link: ET.Element | None = None

    def add_dummy_link(
        self,
        name: str,
        visual_matrix: npt.NDArray[np.float64] | None = None,
        visual_stl: str | None = None,
        visual_color: npt.NDArray[np.float64] | None = None,
    ) -> None:
        link = ET.SubElement(self.xml_root, "link", name=name)
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
        # XXX: We use a low mass because PyBullet consider mass 0 as world fixed
        if self.no_dynamics:
            ET.SubElement(inertial, "mass", value="0")
        else:
            ET.SubElement(inertial, "mass", value="1e-9")
        ET.SubElement(
            inertial, "inertia", ixx="0", ixy="0", ixz="0", iyy="0", iyz="0", izz="0"
        )
        if visual_stl is not None:
            if visual_matrix is None or visual_color is None:
                raise RuntimeError("visual_matrix, visual_stl, and visual_color must all be set if any one are set")

            self.add_stl(
                link,
                visual_matrix,
                visual_stl,
                visual_color,
                name + "_visual",
                "visual",
            )

    def add_dummy_base_link_method(self, name: str) -> None:
        # adds a dummy base_link for ROS users
        ET.SubElement(self.xml_root, "link", name="base_link")
        joint = ET.SubElement(
            self.xml_root, "joint", name="base_link_to_base", type="fixed"
        )
        ET.SubElement(joint, "parent", link="base_link")
        ET.SubElement(joint, "child", link=name)
        ET.SubElement(joint, "origin", rpy="0.0 0 0", xyz="0 0 0")

    def add_fixed_joint(
        self,
        parent: str,
        child: str,
        matrix: npt.NDArray[np.float64],
        name: str | None = None,
    ) -> None:
        if name is None:
            name = parent + "_" + child + "_fixing"

        joint = ET.SubElement(self.xml_root, "joint", name=name, type="fixed")
        add_origin_element(joint, matrix)
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=child)
        ET.SubElement(joint, "axis", xyz="0 0 0")

    def start_link(self, name: str) -> None:
        self._link_name = name
        self.reset_link()

        if self.add_dummy_base_link:
            self.add_dummy_base_link_method(name)
            self.add_dummy_base_link = False
        self._active_link = ET.SubElement(self.xml_root, "link", name=name)

    def end_link(self) -> None:
        if self._active_link is None:
            raise RuntimeError("start_link must be called before end_link")

        mass, com, inertia = self.link_dynamics()

        for node in ["visual", "collision"]:
            if self._mesh[node] is not None:
                if self.mesh_dir is None:
                    raise RuntimeError("mesh_dir must be set before saving meshes")

                if node == "visual" and self._color_mass > 0:
                    color = self._color / self._color_mass
                else:
                    color = np.array([0.5, 0.5, 0.5])

                filename = self._link_name + "_" + node + ".stl"
                stl_combine.save_mesh(self._mesh[node], self.mesh_dir + "/" + filename)
                if self.should_simplify_stls(node):
                    stl_combine.simplify_stl(self.mesh_dir + "/" + filename)
                self.add_stl(
                    self._active_link,
                    np.identity(4),
                    filename,
                    color,
                    self._link_name,
                    node,
                )

        inertial = ET.SubElement(self._active_link, "inertial")
        ET.SubElement(
            inertial, "origin", xyz="%.20g %.20g %.20g" % (com[0], com[1], com[2])
        )
        ET.SubElement(inertial, "mass", value="%.20g" % mass)
        ET.SubElement(
            inertial,
            "inertia",
            ixx="%.20g" % inertia[0, 0],
            ixy="%.20g" % inertia[0, 1],
            ixz="%.20g" % inertia[0, 2],
            iyy="%.20g" % inertia[1, 1],
            iyz="%.20g" % inertia[1, 2],
            izz="%.20g" % inertia[2, 2],
        )

        if self.use_fixed_links:
            visual_elem = ET.SubElement(self._active_link, "visual")
            geometry = ET.SubElement(visual_elem, "geometry")
            ET.SubElement(geometry, "box", size="0 0 0")

        self._active_link = None

        if self.use_fixed_links:
            n = 0
            for visual in self._visuals:
                n += 1
                visual_name = "%s_%d" % (self._link_name, n)
                self.add_dummy_link(visual_name, visual[0], visual[1], visual[2])
                self.add_joint(
                    "fixed",
                    self._link_name,
                    visual_name,
                    np.eye(4),
                    visual_name + "_fixing",
                    None,
                )

    def add_frame(self, name: str, matrix: npt.NDArray[np.float64]) -> None:
        # Adding a dummy link
        self.add_dummy_link(name)

        # Linking it with last link with a fixed link
        self.add_fixed_joint(self._link_name, name, matrix, name + "_frame")

    def add_stl(
        self,
        parent: ET.Element,
        matrix: npt.NDArray[np.float64],
        stl: str,
        color: npt.NDArray[np.float64],
        name: str,
        node: str = "visual",
    ) -> None:
        stl_file = self.package_name.strip("/") + "/" + stl

        material_name = name + "_material"

        element = ET.SubElement(parent, node)
        add_origin_element(element, matrix)
        geometry = ET.SubElement(element, "geometry")
        ET.SubElement(geometry, "mesh", filename=f"package://{stl_file}")

        if node == "visual":
            material = ET.SubElement(element, "material", name=material_name)
            ET.SubElement(
                material,
                "color",
                rgba="%.20g %.20g %.20g 1.0" % (color[0], color[1], color[2]),
            )

    def add_part(
        self,
        matrix: npt.NDArray[np.float64],
        stl: str | None,
        mass: float,
        com: npt.NDArray[np.float64],
        inertia: npt.NDArray[np.float64],
        color: npt.NDArray[np.float64],
        name: str = "",
    ) -> None:
        if self._active_link is None:
            raise RuntimeError("Cannot call addPart before calling start_link")

        if stl is not None:
            if not self.draw_collisions:
                if self.use_fixed_links:
                    self._visuals.append(
                        [matrix, self.package_name + os.path.basename(stl), color]
                    )
                elif self.should_merge_stls("visual"):
                    self.merge_stl(stl, matrix, color, mass)
                else:
                    self.add_stl(
                        self._active_link,
                        matrix,
                        os.path.basename(stl),
                        color,
                        name,
                        "visual",
                    )

            entries = ["collision"]
            if self.draw_collisions:
                entries.append("visual")
            for entry in entries:
                # We don't have pure shape, we use the mesh
                if self.should_merge_stls(entry):
                    self.merge_stl(stl, matrix, color, mass, entry)
                else:
                    self.add_stl(
                        self._active_link,
                        matrix,
                        os.path.basename(stl),
                        color,
                        name,
                        entry,
                    )

        self.add_link_dynamics(matrix, mass, com, inertia)

    def add_joint(
        self,
        joint_type: str,
        link_from: str,
        link_to: str,
        transform: npt.NDArray[np.float64],
        name: str,
        joint_limits: list[float] | None,
        z_axis: list[float] | None = None,
    ) -> None:
        z_axis = [0, 0, 1] if z_axis is None else z_axis

        joint = ET.SubElement(self.xml_root, "joint", name=name, type=joint_type)
        add_origin_element(joint, transform)
        ET.SubElement(joint, "parent", link=link_from)
        ET.SubElement(joint, "child", link=link_to)
        ET.SubElement(joint, "axis", xyz="%.20g %.20g %.20g" % tuple(z_axis))

        limit_elem = ET.SubElement(
            joint,
            "limit",
            effort="%.20g" % self.joint_max_effort_for(name),
            velocity="%.20g" % self.joint_max_velocity_for(name),
        )
        if joint_limits is not None:
            limit_elem.set("lower", "%.20g" % joint_limits[0])
            limit_elem.set("upper", "%.20g" % joint_limits[1])
        ET.SubElement(joint, "joint_properties", friction="0.0")

    def write_to(self, stream: TextIO) -> None:
        stream.write(ET.tostring(self.xml_root, encoding="unicode"))
