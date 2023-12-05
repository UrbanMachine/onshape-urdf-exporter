import math
import sys
from typing import Any

from colorama import Fore, Style

from .config_file import Configuration
from .onshape_api.client import Client

joint_features: dict[str, Any] = {}
configuration_parameters: dict[str, str] = {}


def init(
    client: Client,
    config: Configuration,
    root: dict[str, str],
    workspace_id: str,
    assembly_id: str,
) -> None:
    global configuration_parameters, joint_features

    # Load joint features to get limits later
    if config.version_id == "":
        joint_features = client.get_features(
            config.document_id, workspace_id, assembly_id
        )
    else:
        joint_features = client.get_features(
            config.document_id, config.version_id, assembly_id, type="v"
        )

    # Retrieving root configuration parameters
    configuration_parameters = {}
    parts = root["fullConfiguration"].split(";")
    for part in parts:
        kv = part.split("=")
        if len(kv) == 2:
            configuration_parameters[kv[0]] = kv[1].replace("+", " ")


def read_expression(expression: str) -> float:
    # Expression can itself be a variable from configuration
    # XXX: This doesn't handle all expression, only values and variables
    if expression[0] == "#":
        expression = configuration_parameters[expression[1:]]
    if expression[0:2] == "-#":
        expression = "-" + configuration_parameters[expression[2:]]

    parts = expression.split(" ")

    # Checking the unit, returning only radians and meters
    if parts[1] == "deg":
        return math.radians(float(parts[0]))
    elif parts[1] in ["radian", "rad"]:
        return float(parts[0])
    elif parts[1] == "mm":
        return float(parts[0]) / 1000.0
    elif parts[1] == "m":
        return float(parts[0])
    else:
        print(Fore.RED + "Unknown unit: " + parts[1] + Style.RESET_ALL)
        sys.exit(1)


def read_parameter_value(parameter: dict[str, Any], name: str) -> float:
    # This is an expression
    if parameter["typeName"] == "BTMParameterNullableQuantity":
        return read_expression(parameter["message"]["expression"])
    if parameter["typeName"] == "BTMParameterConfigured":
        message = parameter["message"]
        parameter_value = configuration_parameters[message["configurationParameterId"]]

        for value in message["values"]:
            if value["typeName"] == "BTMConfiguredValueByBoolean":
                boolean_value = parameter_value == "true"
                if value["message"]["booleanValue"] == boolean_value:
                    return read_expression(
                        value["message"]["value"]["message"]["expression"]
                    )
            elif value["typeName"] == "BTMConfiguredValueByEnum":
                if value["message"]["enumValue"] == parameter_value:
                    return read_expression(
                        value["message"]["value"]["message"]["expression"]
                    )
            else:
                print(
                    Fore.RED
                    + "Can't read value of parameter "
                    + name
                    + " configured with "
                    + value["typeName"]
                    + Style.RESET_ALL
                )
                sys.exit(1)

        print(Fore.RED + "Could not find the value for " + name + Style.RESET_ALL)
        sys.exit(1)
    else:
        print(
            Fore.RED
            + "Unknown feature type for "
            + name
            + ": "
            + parameter["typeName"]
            + Style.RESET_ALL
        )
        sys.exit(1)


def get_limits(joint_type: str, name: str) -> tuple[float, float] | None:
    """Gets the limits of a given joint"""
    enabled = False
    minimum, maximum = 0.0, 0.0
    for feature in joint_features["features"]:
        # Find corresponding joint
        if name == feature["message"]["name"]:
            # Find min and max values
            for parameter in feature["message"]["parameters"]:
                if parameter["message"]["parameterId"] == "limitsEnabled":
                    enabled = parameter["message"]["value"]

                if joint_type == "revolute":
                    if parameter["message"]["parameterId"] == "limitAxialZMin":
                        minimum = read_parameter_value(parameter, name)
                    if parameter["message"]["parameterId"] == "limitAxialZMax":
                        maximum = read_parameter_value(parameter, name)
                elif joint_type == "prismatic":
                    if parameter["message"]["parameterId"] == "limitZMin":
                        minimum = read_parameter_value(parameter, name)
                    if parameter["message"]["parameterId"] == "limitZMax":
                        maximum = read_parameter_value(parameter, name)
    if enabled:
        return minimum, maximum
    else:
        if joint_type != "continuous":
            print(
                Fore.YELLOW
                + "WARNING: joint "
                + name
                + " of type "
                + joint_type
                + " has no limits "
                + Style.RESET_ALL
            )
        return None
