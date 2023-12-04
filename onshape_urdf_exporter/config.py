import os
import sys
from sys import exit

import commentjson as json
from colorama import Fore, Style, just_fix_windows_console

just_fix_windows_console()
config = {}

# Loading configuration & parameters
if len(sys.argv) <= 1:
    print(
        Fore.RED + "ERROR: usage: onshape-urdf-exporter {robot_directory}" + Style.RESET_ALL
    )
    print("Read documentation at https://onshape-to-robot.readthedocs.io/")
    exit("")
robot = sys.argv[1]


def configGet(name, default=None, hasDefault=False, valuesList=None):
    global config
    hasDefault = hasDefault or (default is not None)

    if name in config:
        value = config[name]
        if valuesList is not None and value not in valuesList:
            print(
                Fore.RED
                + "ERROR: Value for "
                + name
                + " should be one of: "
                + (",".join(valuesList))
                + Style.RESET_ALL
            )
            exit()
        return value
    else:
        if hasDefault:
            return default
        else:
            print(
                Fore.RED
                + 'ERROR: missing key "'
                + name
                + '" in config'
                + Style.RESET_ALL
            )
            exit()


configFile = robot + "/config.json"
if not os.path.exists(configFile):
    print(
        Fore.RED + "ERROR: The file " + configFile + " can't be found" + Style.RESET_ALL
    )
    exit()
with open(configFile, "r", encoding="utf8") as stream:
    config = json.load(stream)

config["documentId"] = configGet("documentId")
config["versionId"] = configGet("versionId", "")
config["workspaceId"] = configGet("workspaceId", "")
config["drawFrames"] = configGet("drawFrames", False)
config["drawCollisions"] = configGet("drawCollisions", False)
config["assemblyName"] = configGet("assemblyName", False)
config["outputFormat"] = configGet("outputFormat", "urdf")
config["useFixedLinks"] = configGet("useFixedLinks", False)
config["configuration"] = configGet("configuration", "default")
config["ignoreLimits"] = configGet("ignoreLimits", False)

# Using OpenSCAD for simplified geometry
config["pureShapeDilatation"] = configGet("pureShapeDilatation", 0.0)

# Dynamics
config["jointMaxEffort"] = configGet("jointMaxEffort", 1)
config["jointMaxVelocity"] = configGet("jointMaxVelocity", 20)
config["noDynamics"] = configGet("noDynamics", False)

# Ignore list
config["ignore"] = configGet("ignore", [])
config["whitelist"] = configGet("whitelist", None, hasDefault=True)

# Color override
config["color"] = configGet("color", None, hasDefault=True)

# STLs merge and simplification
config["mergeSTLs"] = configGet(
    "mergeSTLs", "no", valuesList=["no", "visual", "collision", "all"]
)
config["maxSTLSize"] = configGet("maxSTLSize", 3)
config["simplifySTLs"] = configGet(
    "simplifySTLs", "no", valuesList=["no", "visual", "collision", "all"]
)

# Post-import commands to execute
config["postImportCommands"] = configGet("postImportCommands", [])

config["outputDirectory"] = robot
config["dynamicsOverride"] = {}

# Add collisions=true configuration on parts
config["useCollisionsConfigurations"] = configGet("useCollisionsConfigurations", True)

# ROS support
config["packageName"] = configGet("packageName", "")
config["addDummyBaseLink"] = configGet("addDummyBaseLink", False)
config["robotName"] = configGet("robotName", "onshape")

# additional XML code to insert
if config["outputFormat"] == "urdf":
    additionalFileName = configGet("additionalUrdfFile", "")
else:
    additionalFileName = configGet("additionalSdfFile", "")

if additionalFileName == "":
    config["additionalXML"] = ""
else:
    with open(robot + additionalFileName, "r", encoding="utf-8") as stream:
        config["additionalXML"] = stream.read()


# Creating dynamics override array
tmp = configGet("dynamics", {})
for key in tmp:
    if tmp[key] == "fixed":
        config["dynamicsOverride"][key.lower()] = {
            "com": [0, 0, 0],
            "mass": 0,
            "inertia": [0, 0, 0, 0, 0, 0, 0, 0, 0],
        }
    else:
        config["dynamicsOverride"][key.lower()] = tmp[key]

# Output directory, making it if it doesn't exists
try:
    os.makedirs(config["outputDirectory"])
except OSError:
    pass


# Checking that versionId and workspaceId are not set on same time
if config["versionId"] != "" and config["workspaceId"] != "":
    print(Style.RED + "You can't specify workspaceId AND versionId")
