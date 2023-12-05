import sys
from pathlib import Path
from typing import Any

from colorama import Fore, Style
from pydantic import BaseModel, ValidationError
from pydantic_yaml import parse_yaml_file_as


class Configuration(BaseModel):
    document_id: str
    version_id: str = ""
    workspace_id: str = ""
    draw_frames: bool = False
    draw_collisions: bool = False
    assembly_name: str = False
    use_fixed_links: bool = False
    configuration: str = "default"
    ignore_limits: bool = False

    # Dynamics
    joint_max_effort: float = 1.0
    joint_max_velocity: float = 20.0
    no_dynamics: bool = False

    # Ignore list
    ignore: list[str] = []
    whitelist: list[str] | None = None

    # Color override
    color: list[float] | None = None

    # STL configuration
    simplify_stls: bool = False

    # Post-import commands to execute
    post_import_commands: list[str] = []

    dynamics_override: dict[str, Any] = {}

    # Add collisions=true configuration on parts
    use_collisions_configurations: bool = True

    # ROS support
    package_name: str = ""
    add_dummy_base_link: bool = False
    robot_name: str = "onshape"

    additional_urdf_file: str = ""
    additional_xml: str = ""

    dynamics: dict[str, Any] = {}

    class Config:
        arbitrary_types_allowed = True

    singleton: "Configuration" = None

    @classmethod
    def from_file(cls, robot_directory: Path) -> "Configuration":
        try:
            cls.singleton = parse_yaml_file_as(
                Configuration, robot_directory / "config.yaml"
            )
        except ValidationError as ex:
            print(f"{Fore.RED}{ex}{Style.RESET_ALL}")
            sys.exit(1)
        return cls.singleton
