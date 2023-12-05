import hashlib
import os
from pathlib import Path
from sys import exit

import commentjson as json
import numpy as np
from colorama import Fore, Style

from .stl_utils import simplify_stl
from .robot_description import RobotURDF

partNames = {}


def main():
    # Loading configuration, collecting occurrences and building robot tree
    from .load_robot import client, config, frames, getOccurrence, occurrences, tree

    robot = RobotURDF(config["robotName"])

    robot.draw_collisions = config["drawCollisions"]
    robot.joint_max_effort = config["jointMaxEffort"]
    robot.simplify_stls = config["simplifySTLs"]
    robot.joint_max_velocity = config["jointMaxVelocity"]
    robot.no_dynamics = config["noDynamics"]
    robot.package_name = config["packageName"]
    robot.addDummyBaseLink = config["addDummyBaseLink"]
    robot.robot_name = config["robotName"]
    robot.additionalXML = config["additionalXML"]
    robot.use_fixed_links = config["useFixedLinks"]
    robot.mesh_dir = config["outputDirectory"]

    def partIsIgnore(name):
        if config["whitelist"] is None:
            return name in config["ignore"]
        else:
            return name not in config["whitelist"]

    # Adds a part to the current robot link

    def addPart(occurrence, matrix):
        part = occurrence["instance"]

        if part["suppressed"]:
            return

        if part["partId"] == "":
            print(
                Fore.YELLOW
                + "WARNING: Part "
                + part["name"]
                + " has no partId"
                + Style.RESET_ALL
            )
            return

        # Importing STL file for this part
        justPart, prefix = extractPartName(part["name"], part["configuration"])

        extra = ""
        if occurrence["instance"]["configuration"] != "default":
            extra = (
                Style.DIM
                + " (configuration: "
                + occurrence["instance"]["configuration"]
                + ")"
            )
        symbol = "+"
        if partIsIgnore(justPart):
            symbol = "-"
            extra += Style.DIM + " / ignoring visual and collision"

        print(
            Fore.GREEN
            + symbol
            + " Adding part "
            + occurrence["instance"]["name"]
            + extra
            + Style.RESET_ALL
        )

        stl_file: Path | None
        if partIsIgnore(justPart):
            stl_file = None
        else:
            stl_file = Path(config["outputDirectory"]) / sanitize_filename(
                prefix + ".stl"
            )
            # shorten the configuration to a maximum number of chars to prevent errors.
            # Necessary for standard parts like screws
            if len(part["configuration"]) > 40:
                shortend_configuration = hashlib.md5(
                    part["configuration"].encode("utf-8")
                ).hexdigest()
            else:
                shortend_configuration = part["configuration"]
            stl = client.part_studio_stl_m(
                part["documentId"],
                part["documentMicroversion"],
                part["elementId"],
                part["partId"],
                shortend_configuration,
            )
            stl_file.write_bytes(stl)
            if config["simplifySTLs"]:
                simplify_stl(stl_file)

            stlMetadata = sanitize_filename(prefix + ".part")
            with open(
                config["outputDirectory"] + "/" + stlMetadata, "w", encoding="utf-8"
            ) as stream:
                json.dump(part, stream, indent=4, sort_keys=True)

        # Obtain metadatas about part to retrieve color
        if config["color"] is not None:
            color = config["color"]
        else:
            metadata = client.part_get_metadata(
                part["documentId"],
                part["documentMicroversion"],
                part["elementId"],
                part["partId"],
                part["configuration"],
            )

            color = [0.5, 0.5, 0.5]

            # XXX: There must be a better way to retrieve the part color
            for entry in metadata["properties"]:
                if (
                    "value" in entry
                    and type(entry["value"]) is dict
                    and "color" in entry["value"]
                ):
                    rgb = entry["value"]["color"]
                    color = np.array([rgb["red"], rgb["green"], rgb["blue"]]) / 255.0

        # Obtain mass properties about that part
        if config["noDynamics"]:
            mass = 0
            com = [0] * 3
            inertia = [0] * 12
        else:
            if prefix in config["dynamicsOverride"]:
                entry = config["dynamicsOverride"][prefix]
                mass = entry["mass"]
                com = entry["com"]
                inertia = entry["inertia"]
            else:
                massProperties = client.part_mass_properties(
                    part["documentId"],
                    part["documentMicroversion"],
                    part["elementId"],
                    part["partId"],
                    part["configuration"],
                )

                if part["partId"] not in massProperties["bodies"]:
                    print(
                        Fore.YELLOW
                        + "WARNING: part "
                        + part["name"]
                        + " has no dynamics (maybe it is a surface)"
                        + Style.RESET_ALL
                    )
                    return
                massProperties = massProperties["bodies"][part["partId"]]
                mass = massProperties["mass"][0]
                com = massProperties["centroid"]
                inertia = massProperties["inertia"]

                if abs(mass) < 1e-9:
                    print(
                        Fore.YELLOW
                        + "WARNING: part "
                        + part["name"]
                        + " has no mass, maybe you should assign a material to it ?"
                        + Style.RESET_ALL
                    )

        pose = occurrence["transform"]
        if robot.relative:
            pose = np.linalg.inv(matrix) * pose

        robot.add_part(pose, stl_file, mass, com, inertia, color, prefix)

    partNames = {}

    def extractPartName(name, configuration):
        parts = name.split(" ")
        del parts[-1]
        basePartName = "_".join(parts).lower()

        # only add configuration to name if its not default and not a very long configuration (which happens for library parts like screws)
        if configuration != "default" and len(configuration) < 40:
            parts += ["_" + configuration.replace("=", "_").replace(" ", "_")]

        return basePartName, "_".join(parts).lower()

    def processPartName(name, configuration, overrideName=None):
        if overrideName is None:
            global partNames
            _, name = extractPartName(name, configuration)

            if name in partNames:
                partNames[name] += 1
            else:
                partNames[name] = 1

            if partNames[name] == 1:
                return name
            else:
                return name + "_" + str(partNames[name])
        else:
            return overrideName

    def buildRobot(tree, matrix):
        occurrence = getOccurrence([tree["id"]])
        instance = occurrence["instance"]
        print(
            Fore.BLUE
            + Style.BRIGHT
            + "* Adding top-level instance ["
            + instance["name"]
            + "]"
            + Style.RESET_ALL
        )

        # Build a part name that is unique but still informative
        link = processPartName(
            instance["name"], instance["configuration"], occurrence["linkName"]
        )

        # Create the link, collecting all children in the tree assigned to this top-level part
        robot.start_link(link)
        for occurrence in occurrences.values():
            if (
                occurrence["assignation"] == tree["id"]
                and "type" in occurrence["instance"]
                and occurrence["instance"]["type"] == "Part"
            ):
                addPart(occurrence, matrix)
        robot.end_link()

        # Adding the frames (linkage is relative to parent)
        if tree["id"] in frames:
            for name, partOrFrame in frames[tree["id"]]:
                if type(partOrFrame) == list:
                    frame = getOccurrence(partOrFrame)["transform"]
                else:
                    frame = partOrFrame

                if robot.relative:
                    frame = np.linalg.inv(matrix) * frame
                robot.add_frame(name, frame)

        # Following the children in the tree, calling this function recursively
        k = 0
        for child in tree["children"]:
            worldAxisFrame = child["axis_frame"]
            zAxis = child["z_axis"]
            jointType = child["jointType"]
            jointLimits = child["jointLimits"]

            if robot.relative:
                axisFrame = np.linalg.inv(matrix) * worldAxisFrame
                childMatrix = worldAxisFrame
            else:
                # In SDF format, everything is expressed in the world frame, in this case
                # childMatrix will be always identity
                axisFrame = worldAxisFrame
                childMatrix = matrix

            subLink = buildRobot(child, childMatrix)
            robot.add_joint(
                jointType,
                link,
                subLink,
                axisFrame,
                child["dof_name"],
                jointLimits,
                zAxis,
            )

        return link

    # Start building the robot
    buildRobot(tree, np.matrix(np.identity(4)))

    print(
        "\n"
        + Style.BRIGHT
        + "* Writing "
        + robot.ext.upper()
        + " file"
        + Style.RESET_ALL
    )
    with open(
        config["outputDirectory"] + "/robot." + robot.ext, "w", encoding="utf-8"
    ) as stream:
        robot.write_to(stream)

    if len(config["postImportCommands"]):
        print(
            "\n" + Style.BRIGHT + "* Executing post-import commands" + Style.RESET_ALL
        )
        for command in config["postImportCommands"]:
            print("* " + command)
            os.system(command)


def sanitize_filename(value: str) -> str:
    """Escapes characters that are invalid in filenames on Linux filesystems and NTFS.
    The escape sequences mimic URL encoding.

    Based on info from: https://stackoverflow.com/a/31976060
    """
    invalid_chars = [chr(i) for i in range(32)] + [
        "<",
        ">",
        ":",
        '"',
        "/",
        "\\",
        "|",
        "?",
        "*",
    ]

    for char in invalid_chars:
        value = value.replace(char, hex(ord(char)).replace("0x", "%"))

    return value


if __name__ == "__main__":
    main()
