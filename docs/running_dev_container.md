<!-- ---------------------------------------------------------------------- -->
## **Running The Dev-Container**
 
If you have your [VSCode](https://code.visualstudio.com/) with the right extensions, and if you have Docker and Docker-compose installed in your system, when you open the project's main folder you'll see a window on the bottom right corner, click in "reopen in container" button, if you don't see anything press `Ctrl+Shift+P` and type `Remote-Containers: Rebuild and Reopen in container` or `Docker-images: Build Image` option. When the container is opened, and executed for the first time or when there are changes on it, you can go for a walk because the building image process will start and it'll take a while due to the installation of all packages and dependencies of the dev-environment as [ROS2](https://index.ros.org/doc/ros2/), [OpenCV](https://opencv.org/), [Python](https://www.python.org/), and more stuff related to, while the process is completed here are some videos of [puppies](https://www.youtube.com/watch?v=mRf3-JkwqfU). You can see at any time the logs of the building process clicking in `Starting with Dev Container` on the bottom right corner. When the process is done you'll see some messages of process succeed.
 
<img src="https://user-images.githubusercontent.com/43115782/87437367-d5806200-c5b3-11ea-9bf2-836e45f46ed8.gif" alt="building_dev-container" width="1200">
 
### Oh no! You found out that the dev-container is not working? This is your first task :sunglasses: Refer to the [homework.md](./homework.md) file for more details.

When the process is done you can open a terminal in the dev-container going to the menu bar `Terminal` and then `New Terminal`. Congratulations now you have everything that we use for our deployments.
 

<br />

<!-- ---------------------------------------------------------------------- -->
## **Architecture**
 
Find the distribution of final project in the next list:
 
- **[rover](../rover):** Main folder where most of the source code is located
  - **[configs](../rover/configs):** Path rover config files
     - [*env_vars.sh:*](/rover/configs/env_vars.sh) local environment variables
     - [*nodes_launch.sh:*](/rover/configs/nodes_launch.sh) file describing which nodes launch or not
  - **[ros2/src](../rover/ros2/src):** Development workspace & ROS 2 packages
  - **[media](../rover/media):** Media files for project such images, and audios 
  - **[scripts](../scripts):** Scripts for the project
     - [*startRobotics.sh:*](/scripts/startRobotics.sh) bash script to run stack of the project
     - [*alias_script.sh:*](/scripts/alias_script.sh) bash script to setup aliases for the project


Only some files are listed, and explained (most important).
 
<br />
 

<!-- ---------------------------------------------------------------------- -->
## **Running The Project Stack**
 
Find a brief explanation on how to run our stack in your IDE and the explanation of the launch file, and config files as the key to managing which nodes are going to be launched and how they're going to work.
 
In order to launch locally (Inside your IDE), please locate into the `scripts/` folder in the dev-container terminal and update submodules:

     $ git submodule update --init --recursive

Then, execute the following prompt command:
 
     $ bash scripts/startRobotics.sh

*Note:* if you've already launch and compile the whole stack and there's no a *hot* modification inside the stack, it's possible to avoid the entire compiling step running:

     $ bash scripts/startRobotics.sh  # [args: -b {build}, -l {launch}]

> There are some arguments to choose whether build/launch nodes or not.
>- Build: ```bash startRobotics.sh -b 0``` or ```bash startRobotics.sh --build 0``` 1 to build 0 to not. IMPORTANT: only the nodes with the field set to 1 in some of the [nodes_launch.sh:](../rover/configs/nodes_launch.sh) will be compiled.

This bash performs the following steps to launch the *Robotics* stack:
 
1. Sources the [`env_vars.sh`](../rover/configs/env_vars.sh) which contains the *Kiwibot* local environment variables.
2. Build the nodes you've selected in the [*nodes_launch.sh:*](../rover/configs/nodes_launch.sh) file
3. Launch ros2 with the specified node in [*nodes_launch.sh:*](../rover/configs/nodes_launch.sh)

For ROS 2 development workspace

1. Sources ROS Iron and clean the older development workspace (If enabled).
2. Builds the development workspace at [`rover/ros2/`](rover/ros2)
3. Sources the resulting setup in the install folder `. install/setup.bash`
4. Executes [`ros2 launch /configs/rover.launch.py`](../rover/configs/rover.launch.py)

You can compile and launch everything by your own if you already have a background and experience with ROS/ROS2, but for those who want everything easy, and fast the bash script will set up and run everything for you. With the [``startRobotics.sh``](../scripts/startRobotics.sh) bash script you can run the stack of the project, this file has all instruction to download third-party packages, other required dependencies if they're missing, and setup, source, and run the ros2 workspace, compile and launch the nodes specified in the file [*nodes_launch.sh:*](/rover/configs/nodes_launch.sh).

<!-- ---------------------------------------------------------------------- -->
## Troubleshooting

*Note (Audio is not reproducing IN THE SOLUTION VERSION):* if you are having troubles getting audio from the virtual environment, please create a issue with the error description. But also check the audio device is not busy, or try connecting and disconnecting headsets in the audio port.

*Note (Launching specific nodes):* Remember, if you want to launch or build just a single node or some of them or the whole stack, go to the file [*nodes_launch.sh:*](/rover/configs/nodes_launch.sh) and set the node to 1 to launch and 0 to don't launch. This could be useful when you are testing some nodes or just a single node.

<br />