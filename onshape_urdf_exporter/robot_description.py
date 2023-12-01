import math
import os
import uuid
from typing import TextIO
from xml.etree import ElementTree as ET

import numpy as np

from . import stl_combine


def rotationMatrixToEulerAngles(R):
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


def origin(matrix):
    urdf = '<origin xyz="%.20g %.20g %.20g" rpy="%.20g %.20g %.20g" />'
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]
    rpy = rotationMatrixToEulerAngles(matrix)

    return urdf % (x, y, z, rpy[0], rpy[1], rpy[2])


def add_origin_element(parent: ET.Element, matrix) -> None:
    # TODO: Make this not use '%' for formatting
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]
    xyz_str = "%.20g %.20g %.20g" % (x, y, z)

    r, p, y = rotationMatrixToEulerAngles(matrix)
    rpy_str = "%.20g %.20g %.20g" % (r, p, y)

    ET.SubElement(parent, "origin", xyz=xyz_str, rpy=rpy_str)


def pose(matrix, frame=""):
    sdf = "<pose>%.20g %.20g %.20g %.20g %.20g %.20g</pose>"
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]
    rpy = rotationMatrixToEulerAngles(matrix)

    if frame != "":
        sdf = '<frame name="' + frame + '_frame">' + sdf + "</frame>"

    return sdf % (x, y, z, rpy[0], rpy[1], rpy[2])


class RobotDescription(object):
    def __init__(self, name):
        self.drawCollisions = False
        self.relative = True
        self.mergeSTLs = "no"
        self.mergeSTLsCollisions = False
        self.useFixedLinks = False
        self.simplifySTLs = "no"
        self.maxSTLSize = 3
        self.xml_root: ET.Element | None = None
        self.jointMaxEffort = 1
        self.jointMaxVelocity = 10
        self.noDynamics = False
        self.packageName = ""
        self.addDummyBaseLink = False
        self.robotName = name
        self.meshDir = None

    def shouldMergeSTLs(self, node):
        return self.mergeSTLs == "all" or self.mergeSTLs == node

    def shouldSimplifySTLs(self, node):
        return self.simplifySTLs == "all" or self.simplifySTLs == node

    def jointMaxEffortFor(self, jointName):
        if isinstance(self.jointMaxEffort, dict):
            if jointName in self.jointMaxEffort:
                return self.jointMaxEffort[jointName]
            else:
                return self.jointMaxEffort["default"]
        else:
            return self.jointMaxEffort

    def jointMaxVelocityFor(self, jointName):
        if isinstance(self.jointMaxVelocity, dict):
            if jointName in self.jointMaxVelocity:
                return self.jointMaxVelocity[jointName]
            else:
                return self.jointMaxVelocity["default"]
        else:
            return self.jointMaxVelocity

    def resetLink(self):
        self._mesh = {"visual": None, "collision": None}
        self._color = np.array([0.0, 0.0, 0.0])
        self._color_mass = 0
        self._link_childs = 0
        self._visuals = []
        self._dynamics = []

    def addLinkDynamics(self, matrix, mass, com, inertia):
        # Inertia
        I = np.matrix(np.reshape(inertia[:9], (3, 3)))
        R = matrix[:3, :3]
        # Expressing COM in the link frame
        com = np.array((matrix * np.matrix([com[0], com[1], com[2], 1]).T).T)[0][:3]
        # Expressing inertia in the link frame
        inertia = R * I * R.T

        self._dynamics.append({"mass": mass, "com": com, "inertia": inertia})

    def mergeSTL(self, stl, matrix, color, mass, node="visual"):
        if node == "visual":
            self._color += np.array(color) * mass
            self._color_mass += mass

        m = stl_combine.load_mesh(stl)
        stl_combine.apply_matrix(m, matrix)

        if self._mesh[node] is None:
            self._mesh[node] = m
        else:
            self._mesh[node] = stl_combine.combine_meshes(self._mesh[node], m)

    def linkDynamics(self):
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
            inertia += (
                dynamic["inertia"]
                + (np.dot(r, r) * identity - p.T * p) * dynamic["mass"]
            )

        return mass, com, inertia


class RobotURDF(RobotDescription):
    def __init__(self, name):
        super().__init__(name)
        self.ext = "urdf"
        self.xml_root = ET.Element("robot", name=self.robotName)
        self._active_link: ET.Element | None = None

    def addDummyLink(self, name, visualMatrix=None, visualSTL=None, visualColor=None):
        link = ET.SubElement(self.xml_root, "link", name=name)
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
        # XXX: We use a low mass because PyBullet consider mass 0 as world fixed
        if self.noDynamics:
            ET.SubElement(inertial, "mass", value="0")
        else:
            ET.SubElement(inertial, "mass", value="1e-9")
        ET.SubElement(
            inertial, "inertia", ixx="0", ixy="0", ixz="0", iyy="0", iyz="0", izz="0"
        )
        if visualSTL is not None:
            self.addSTL(
                link, visualMatrix, visualSTL, visualColor, name + "_visual", "visual"
            )

    def addDummyBaseLinkMethod(self, name):
        # adds a dummy base_link for ROS users
        ET.SubElement(self.xml_root, "link", name="base_link")
        joint = ET.SubElement(
            self.xml_root, "joint", name="base_link_to_base", type="fixed"
        )
        ET.SubElement(joint, "parent", link="base_link")
        ET.SubElement(joint, "child", link=name)
        ET.SubElement(joint, "origin", rpy="0.0 0 0", xyz="0 0 0")

    def addFixedJoint(self, parent, child, matrix, name=None):
        if name is None:
            name = parent + "_" + child + "_fixing"

        joint = ET.SubElement(self.xml_root, "joint", name=name, type="fixed")
        add_origin_element(joint, matrix)
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=child)
        ET.SubElement(joint, "axis", xyz="0 0 0")

    def start_link(self, name) -> None:
        self._link_name = name
        self.resetLink()

        if self.addDummyBaseLink:
            self.addDummyBaseLinkMethod(name)
            self.addDummyBaseLink = False
        self._active_link = ET.SubElement(self.xml_root, "link", name=name)

    def end_link(self) -> None:
        if self._active_link is None:
            raise RuntimeError("start_link must be called before end_link")

        mass, com, inertia = self.linkDynamics()

        for node in ["visual", "collision"]:
            if self._mesh[node] is not None:
                if node == "visual" and self._color_mass > 0:
                    color = self._color / self._color_mass
                else:
                    color = [0.5, 0.5, 0.5]

                filename = self._link_name + "_" + node + ".stl"
                stl_combine.save_mesh(self._mesh[node], self.meshDir + "/" + filename)
                if self.shouldSimplifySTLs(node):
                    stl_combine.simplify_stl(
                        self.meshDir + "/" + filename, self.maxSTLSize
                    )
                self.addSTL(
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

        if self.useFixedLinks:
            visual = ET.SubElement(self._active_link, "visual")
            geometry = ET.SubElement(visual, "geometry")
            ET.SubElement(geometry, "box", size="0 0 0")

        self._active_link = None

        if self.useFixedLinks:
            n = 0
            for visual in self._visuals:
                n += 1
                visual_name = "%s_%d" % (self._link_name, n)
                self.addDummyLink(visual_name, visual[0], visual[1], visual[2])
                self.addJoint(
                    "fixed",
                    self._link_name,
                    visual_name,
                    np.eye(4),
                    visual_name + "_fixing",
                    None,
                )

    def addFrame(self, name, matrix):
        # Adding a dummy link
        self.addDummyLink(name)

        # Linking it with last link with a fixed link
        self.addFixedJoint(self._link_name, name, matrix, name + "_frame")

    def addSTL(self, parent: ET.Element, matrix, stl, color, name, node="visual"):
        stl_file = self.packageName.strip("/") + "/" + stl

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

    def addPart(self, matrix, stl, mass, com, inertia, color, name=""):
        if self._active_link is None:
            raise RuntimeError("Cannot call addPart before calling start_link")

        if stl is not None:
            if not self.drawCollisions:
                if self.useFixedLinks:
                    self._visuals.append(
                        [matrix, self.packageName + os.path.basename(stl), color]
                    )
                elif self.shouldMergeSTLs("visual"):
                    self.mergeSTL(stl, matrix, color, mass)
                else:
                    self.addSTL(
                        self._active_link,
                        matrix,
                        os.path.basename(stl),
                        color,
                        name,
                        "visual",
                    )

            entries = ["collision"]
            if self.drawCollisions:
                entries.append("visual")
            for entry in entries:
                # We don't have pure shape, we use the mesh
                if self.shouldMergeSTLs(entry):
                    self.mergeSTL(stl, matrix, color, mass, entry)
                else:
                    self.addSTL(
                        self._active_link,
                        matrix,
                        os.path.basename(stl),
                        color,
                        name,
                        entry,
                    )

        self.addLinkDynamics(matrix, mass, com, inertia)

    def addJoint(
        self, jointType, linkFrom, linkTo, transform, name, jointLimits, zAxis=[0, 0, 1]
    ):
        joint = ET.SubElement(self.xml_root, "joint", name=name, type=jointType)
        add_origin_element(joint, transform)
        ET.SubElement(joint, "parent", link=linkFrom)
        ET.SubElement(joint, "child", link=linkTo)
        ET.SubElement(joint, "axis", xyz="%.20g %.20g %.20g" % tuple(zAxis))

        limit_elem = ET.SubElement(
            joint,
            "limit",
            effort="%.20g" % self.jointMaxEffortFor(name),
            velocity="%.20g" % self.jointMaxVelocityFor(name),
        )
        if jointLimits is not None:
            limit_elem.set("lower", "%.20g" % jointLimits[0])
            limit_elem.set("upper", "%.20g" % jointLimits[1])
        ET.SubElement(joint, "joint_properties", friction="0.0")

    def finalize(self):
        # TODO: Figure out how to add this back
        # self.append(self.additionalXML)
        pass

    def write_to(self, stream: TextIO) -> None:
        stream.write(ET.tostring(self.xml_root, encoding="unicode"))
