#!/usr/bin/env python3

# =============================================================================
"""
Code Information:
	Kiwibot / Computer & Ai Vision Team

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.

    Examples:
    https://github.com/ros2/launch/blob/a89671962220c8691ea4f128717bca599c711cda/launch/examples/launch_counters.py

"""

# =============================================================================
import inspect
import os
from collections import deque
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart

# ============================================================================


class bcolors:
    LOG = {
        "WARN": ["\033[33m", "WARN"],
        "ERROR": ["\033[91m", "ERROR"],
        "OKGREEN": ["\033[32m", "INFO"],
        "INFO": ["\033[0m", "INFO"],  # ['\033[94m', "INFO"],
        "BOLD": ["\033[1m", "INFO"],
        "GRAY": ["\033[90m", "INFO"],
    }
    BOLD = "\033[1m"
    ENDC = "\033[0m"
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    GRAY = "\033[90m"
    UNDERLINE = "\033[4m"


def printlog(msg, msg_type="INFO", flush=True):
    org = os.path.splitext(os.path.basename(inspect.stack()[1][1]))[0].upper()
    caller = inspect.stack()[1][3].upper()
    _str = "[{}][{}][{}]: {}".format(bcolors.LOG[msg_type][1], org, caller, msg)
    print(bcolors.LOG[msg_type][0] + _str + bcolors.ENDC, flush=flush)


def order_processes(process_dict: dict) -> dict:
    """!
    Order process according to the launch order, result dict filtered by "launch" key
    @param process_dict, process dictionary to be launched dictionary must look like:
        {
            PROCESS_KEY: {
                "launch": boolean,
                "launch_after": OTHER_PROCESS_KEY,
                # any other keys, they will not be modified
            },
    sorting process is performed in the following stages:
    1. Discard processes with launch tag in false
    2. Construct a list of processes with dependencies, if the dependency will not be launched  the dependency will not be added to the list
    3. Push processes without dependencies at he start of the output
    4. Iterate in the process list and add it in the output if its dependencies are already added.
    5. iterate in the process list until all processes are added
    @return ordered process dict, according to the 'launch_after' key
    """
    # Initialize a dictionary to store the dependencies
    dependencies = {
        key: set() for key, details in process_dict.items() if details["launch"]
    }
    process_number = len(dependencies)

    # Populate the dependencies based on the 'launch_after' key
    for process, details in process_dict.items():
        if details["launch"] and "launch_after" in details:
            before_process = details["launch_after"]
            if process_dict[before_process]["launch"]:
                dependencies[process].add(details["launch_after"])
            else:
                del details["launch_after"]

    # Create a list to store the sorted processes
    sorted_processes = []
    # Create a set to track visited nodes
    visited = set()
    # Create a deque for the processes with no dependencies
    no_dependency_processes = deque(
        [process for process, deps in dependencies.items() if not deps]
    )

    while no_dependency_processes:
        process = no_dependency_processes.popleft()
        sorted_processes.append(process)
        visited.add(process)

        # Remove the process from the dependencies of other processes
        for other_process, deps in dependencies.items():
            if process in deps:
                deps.remove(process)
                # If the other process has no more dependencies, add it to the deque
                if not deps and other_process not in visited:
                    no_dependency_processes.append(other_process)

    # Check for cyclic dependencies
    if len(sorted_processes) != process_number:
        raise ValueError("Cyclic dependency detected among the processes")

    # Return the ordered list of processes
    return {process: process_dict[process] for process in sorted_processes}


def read_node_launch(default_nodes, default_yml_file="nodes_launch.sh"):
    CONF_PATH = os.path.dirname(os.path.abspath(__file__))
    FILE_PATH = os.path.join(CONF_PATH, "configs", default_yml_file)

    if not os.path.exists(FILE_PATH):
        try:
            with open(FILE_PATH, "w") as outfile:
                yaml.dump(default_nodes, outfile, default_flow_style=False)
            printlog(msg="Nodes launch file created", msg_type="WARN")

        except Exception as e:
            printlog(
                msg="Error creating nodes launch file: {}".format(e), msg_type="ERROR"
            )

        return default_nodes

    else:
        try:
            with open(FILE_PATH, "r") as stream:
                default_nodes = yaml.full_load(stream)
            printlog(
                msg="Nodes local launch file {}".format(default_yml_file),
                msg_type="OKGREEN",
            )

        except Exception as e:
            printlog(
                msg="Error reading nodes launch file: {}".format(e), msg_type="ERROR"
            )

        return default_nodes


def create_nodes_launch_file(default_nodes, file_path):
    with open(file_path, "w") as outfile:
        for node_name, node_data in default_nodes.items():
            if "package" in node_data:
                launch = node_data["launch"]
                package = node_data["package"]
                skip_build = node_data["skip_build"]
                outfile.write(
                    f"export {node_name}={launch} # {package} skip:{skip_build}\n"
                )


def read_and_set_environment(default_nodes, file_path, all_nodes):
    try:
        with open(file_path, "r") as stream:
            for line in stream:
                # Remove comments from the line (anything after the # character)
                line = line.split("#")[0].strip()
                # Remove the 'export' keyword and spaces before the variable name
                line = line.replace("export ", "").strip()
                if "=" in line:
                    key, value = line.split("=", 1)  # Split at the first = sign only
                    os.environ[key.strip()] = value.strip()
                    if not key.strip() in default_nodes:
                        default_nodes[key.strip()] = all_nodes[key.strip()]
                    default_nodes[key.strip()]["launch"] = int(value.strip())
        printlog(
            msg=f"Nodes local launch file {file_path}",
            msg_type="OKGREEN",
        )
    except Exception as e:
        printlog(msg=f"Error reading nodes launch file: {e}", msg_type="ERROR")


def create_node_launch(default_nodes, default_export_file="nodes_launch.sh"):
    CONF_PATH = Path(os.path.abspath(__file__))
    FILE_PATH = os.path.join(CONF_PATH.parent.parent, "configs", default_export_file)

    if not os.path.exists(FILE_PATH):
        create_nodes_launch_file(default_nodes, FILE_PATH)
        printlog(msg="Nodes launch file created", msg_type="WARN")
        # exit()

    return default_nodes, FILE_PATH


def generate_launch_description():

    ld = LaunchDescription(
        [
            LogInfo(msg="Launching your amazing ROS2 project..."),
        ]
    )

    # -------------------------------------------------------------------------
    # Default nodes to launch
    respawn_nodes = bool(int(os.getenv(key="RESPAWN_NODES", default=1)))
    respawn_delay = float(os.getenv(key="RESPAWN_DELAY", default=5))
    nodes_launch = {
        # ---------------------------------------------------------------------
        # Interfaces
        "NODE_INTERFACES": {
            "node_executable": "interfaces_node",
            "exec_name": "interfaces_node",
            "package": "interfaces",
        },
        # ---------------------------------------------------------------------
        # Fail detection
        "NODE_FAIL_DETECTION_CPP": {
            "node_executable": "fail_detection_node",
            "exec_name": "fail_detection_cpp",
            "package": "fail_detection_cpp",
        },
        "NODE_FAIL_DETECTION_PY": {
            "node_executable": "fail_detection",
            "exec_name": "fail_detection",
            "package": "fail_detection_py",
        },
        # ---------------------------------------------------------------------
        # Rosboard
        "NODE_ROSBOARD": {
            "node_executable": "rosboard_node",
            "exec_name": "rosboard",
            "node_name": "rosboard_node",
            "package": "rosboard",
            "parameters": [
                {
                    "port": int(os.getenv("ROSBOARD_PORT", "80")),
                    "resize_images": bool(int(os.getenv("ROSBOARD_IMAG_RESIZE", "0"))),
                    "max_allowed_latency": int(
                        os.getenv("ROSBOARD_MAX_ALLOWED_LATENCY", "10000")
                    ),
                    "foxglove_uri": os.getenv(
                        "ROSBOARD_FOXGLOVE_URI", "https://foxglove.kiwibot.com"
                    ),
                    "foxglove_layout_uri": os.getenv(
                        "ROSBOARD_FOXGLOVE_LAYOUT_URI",
                        "https://storage.googleapis.com/autonomy-vision/blackfox/layouts/auto_layout_mqtt_heartbeat.json",
                    ),
                    "sensor_msgs/msg/Image": int(
                        os.getenv("ROSBOARD_IMAGE_TOPICS_MAX_RATE", 5)
                    ),
                    "sensor_msgs/msg/PointCloud2": int(
                        os.getenv("ROSBOARD_POINTCLOUD_TOPICS_MAX_RATE", 4)
                    ),
                }
            ],
        },
    }

    nodes = nodes_launch
    # Add 'launch' and 'respawn' arguments to nodes kwargs
    for node_name, node_kwargs in nodes.items():
        if "launch" not in node_kwargs:
            # Add launch parameter using env variable based on its name
            node_kwargs["launch"] = int(os.getenv(key=node_name, default=0))
        # Only add respawn to pure nodes, not from launchfiles
        if "from_launch" not in node_kwargs:
            node_kwargs["respawn"] = respawn_nodes
            node_kwargs["respawn_delay"] = respawn_delay

        if "skip_build" not in node_kwargs:
            # Add skip_build parameter
            node_kwargs["skip_build"] = 0

    nodes, file_path = create_node_launch(
        default_nodes=nodes,
        default_export_file="nodes_launch.sh",
    )
    read_and_set_environment(
        default_nodes=nodes, file_path=file_path, all_nodes=nodes_launch
    )

    # -------------------------------------------------------------------------
    # Get launch description

    # Print nodes to launch
    srt_launched = "\n\nLAUNCHED:"
    srt_launched_exe = ""
    srt_no_launched = "\n\nNO LAUNCHED:" + "\033[93m"
    srt_no_launched_exe = ""
    ljust = 18

    for key, node_args in nodes.items():
        if node_args["launch"]:
            if "exec_name" in node_args.keys():
                srt_launched = srt_launched + "\n\t[package]: {} \t[Node]: {}".format(
                    node_args["package"][:10].ljust(ljust),
                    node_args["exec_name"],
                )
            elif "node_executable" in node_args.keys():
                srt_launched_exe = (
                    srt_launched_exe
                    + "\n\t[Package]: {} \t[Executable]: {}".format(
                        node_args["package"].ljust(ljust),
                        node_args["node_executable"],
                    )
                )
            else:
                srt_launched_exe = srt_launched_exe + "\n\t[Package]: {}".format(
                    node_args.get("package"),
                )
        else:
            if "exec_name" in node_args.keys():
                srt_no_launched = (
                    srt_no_launched
                    + "\n\t[package]: {} \t[Node]: {}".format(
                        node_args["package"][:10].ljust(ljust),
                        node_args["exec_name"],
                    )
                )
            elif "node_executable" in node_args.keys():
                srt_no_launched_exe = (
                    srt_no_launched_exe
                    + "\n\t[Package]: {} \t[Executable]: {}".format(
                        node_args["package"].ljust(ljust),
                        node_args["node_executable"],
                    )
                )
            else:
                srt_no_launched_exe = srt_no_launched_exe + "\n\t[Package]: {}".format(
                    node_args.get("package"),
                )

    ld = LaunchDescription(
        [
            LogInfo(
                msg=srt_launched
                + srt_launched_exe
                + srt_no_launched
                + srt_no_launched_exe
                + "\033[0m"
                + "\n"
            ),
        ]
    )

    nodes = order_processes(nodes)
    actions = dict()

    # -------------------------------------------------------------------------
    for key, node_args in nodes.items():
        # This is for C++ packages
        if "from_launch" in node_args.keys():
            if (not node_args["from_launch"]) or ("package" not in node_args.keys()):
                continue

            launch_dir = get_package_share_directory("{}".format(node_args["package"]))
            launch_arguments = node_args.get("launch_arguments", None)
            included_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    launch_dir + "/launch/{}.py".format(node_args["file"])
                ),
                launch_arguments=(
                    launch_arguments.items() if launch_arguments else launch_arguments
                ),
            )

            launch_action = (
                RegisterEventHandler(
                    OnProcessStart(
                        target_action=actions[node_args["launch_after"]],
                        on_start=[
                            TimerAction(
                                period=node_args.get("delay", 0.0),
                                actions=[included_launch],
                            )
                        ],
                        handle_once=True,
                    )
                )
                if "launch_after" in node_args
                else TimerAction(
                    period=node_args.get("delay", 0.0),
                    actions=[included_launch],
                )
            )

            actions[key] = included_launch
            ld.add_action(launch_action)

        # For python packages
        elif "from_file" in node_args.keys():
            included_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.dirname(os.path.abspath(__file__))
                    + "/{}.py".format(node_args["file"])
                ),
            )

            launch_action = (
                RegisterEventHandler(
                    OnProcessStart(
                        target_action=actions[node_args["launch_after"]],
                        on_start=[
                            TimerAction(
                                period=node_args.get("delay", 0.0),
                                actions=[included_launch],
                            )
                        ],
                        handle_once=True,
                    )
                )
                if "launch_after" in node_args
                else TimerAction(
                    period=node_args.get("delay", 0.0),
                    actions=[included_launch],
                )
            )

            actions[key] = included_launch
            ld.add_action(launch_action)

        else:
            # Add arguments for log level based on its key, node name identifier
            # Ex: NODE_DATA_CAPTURE -> NODE_DATA_CAPTURE_LOG_LEVEL
            log_level_var = f"{key}_LOG_LEVEL"
            log_level_arguments = [
                "--ros-args",
                "--log-level",
                os.getenv(log_level_var, os.getenv("RCUTILS_LOGGING_LEVEL", "INFO")),
            ]
            # If node already has arguments, just add them
            if "arguments" in node_args:
                node_args["arguments"] += log_level_arguments
            else:
                node_args["arguments"] = log_level_arguments

            prefix_var = f"{key}_PREFIX"
            extra_prefix = os.getenv(prefix_var, "")
            nodes_prefix = os.getenv("NODES_PREFIX", "")
            if not extra_prefix:
                extra_prefix = nodes_prefix

            if extra_prefix:
                prefix = f"bash -c '{extra_prefix}; stdbuf -o L $0 $@'"
            else:
                prefix = "stdbuf -o L"

            node = Node(
                executable=node_args["node_executable"],
                name=node_args.get("node_name", None),
                exec_name=node_args.get("exec_name", node_args["node_executable"]),
                package=node_args["package"],
                output="screen",
                respawn=node_args.get("respawn", False),
                respawn_delay=node_args.get("respawn_delay", None),
                arguments=node_args.get("arguments", None),
                prefix=prefix,
                emulate_tty=True,
                parameters=node_args.get("parameters", None),
                namespace=node_args.get("namespace", None),
                remappings=node_args.get("remappings", None),
            )

            node_action = TimerAction(
                period=node_args.get("delay", 0.0),
                actions=[node],
            )

            actions[key] = node
            ld.add_action(node_action)

    return ld

    # ============================================================================
