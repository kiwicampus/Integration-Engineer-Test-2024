#!/bin/bash
# /etc/init.d/startBot

# -----------------------------------------------------------------------------
# clear
# exit script on any error

set -e

function parse_yaml {
   local s='[[:space:]]*' w='[a-zA-Z0-9_]*' fs=$(echo @|tr @ '\034')
   sed -ne "s|^\($s\):|\1|" \
        -e "s|^\($s\)\($w\)$s:$s[\"']\(.*\)[\"']$s\$|\1$fs\2$fs\3|p" \
        -e "s|^\($s\)\($w\)$s:$s\(.*\)$s\$|\1$fs\2$fs\3|p"  $1 |
   awk -F$fs '{
    indent = length($1)/2;
    vname[indent] = $2;
    for (i in vname) {if (i > indent) {delete vname[i]}}
    
    if (length($3) > 0) {
        vn=""; for (i=0; i<indent; i++) {vn=(vn)(vname[i])("_");}
        printf("%s%s=%s\n", vn, $2, $3);
    }
}'
}

BUILD=1
LAUNCH=1
for i in "$@"
do
    case $i in
        -b|--build) BUILD="${2:-1}"
        ;;
        -l|--launch) LAUNCH="${2:-1}"
        ;;
        *)
        ;;
    esac
    shift
done

# -----------------------------------------------------------------------------
# remove empty log files, which are created for no reason (We dont know yet)

find . -name "*.log" -type f -delete

#  ----------------------------------------------------------------------
# Starting

echo  "Starting Robot"
cd /workspace/rover

#  ----------------------------------------------------------------------
# Source enviroment variables

source "${PWD%}/configs/env_vars.sh"

#  ----------------------------------------------------------------------
# Delete previous workspaces

if [ "$DELETE_BUILD" = "1" ] 
then
  echo  [WARN]: "ROS2 Removing old shit ... "
  rm -r ${PWD%}/ros2/install || true
  rm -r ${PWD%}/ros2/build || true
  rm -r ${PWD%}/ros2/log || true
fi

#  ----------------------------------------------------------------------
#  Build ROS2 packages

cd ${PWD%}/ros2 
. /opt/ros/${ROS_DISTRO}/setup.sh

# Extract nodes to build
TO_LAUNCH_FILE="/workspace/rover/configs/nodes_launch.sh"
source $TO_LAUNCH_FILE || true

if [ ! "$BUILD" == "0" ]; then
  echo  "[INFO]: ROS2 Building new stuff ... "

  # Source nodes to be launch
  # To store the packages to compile
  nodes_packages=() 
  # To track seen packages
  declare -A seen_packages=()  

  # Extract packages from the export file. The data inside the file should be in the format: export NODE_XXX=1 # my_pkg skip:0
  while IFS= read -r line || [[ -n "$line" ]]; do
      # Process each line, even if it doesn't end with a line jump
      if [[ "$line" =~ ^export\ (.+)=1\ #\ (.+)\ skip:([01]) ]]; then
          skip_value="${BASH_REMATCH[3]}"
          if [[ "$skip_value" == "0" ]]; then
              package_name="${BASH_REMATCH[2]}"
              if [[ -z "${seen_packages[$package_name]}" ]]; then
                  nodes_packages+=("$package_name")
                  seen_packages["$package_name"]=1  # Mark the package as seen
              fi
          fi
      fi
  done < "$TO_LAUNCH_FILE"

  if [ ! "${#nodes_packages[@]}" == "0" ] ;then
    echo "[INFO]:Packages to build: ${nodes_packages[@]}"
    colcon build --symlink-install --packages-up-to ${nodes_packages[@]}
  else
    echo "[ERROR]: No Launch field set to any node" 
  fi
  echo  "[INFO]: ROS2 Build successful ... "
fi
echo  "[INFO]: ROS2 sourcing ... "
source /workspace/rover/ros2/install/setup.bash || true

if [ ! "$LAUNCH" == "0" ]; then
  #  ---------------------------------------------------------------------
  #  ROS2 Launching
  echo  "[INFO]: ROS2 launching ... "
  ros2 launch "/workspace/rover/configs/rover.launch.py"
fi

#  ----------------------------------------------------------------------

exit 0
