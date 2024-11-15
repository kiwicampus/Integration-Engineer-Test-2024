set -e

# Script to automatically autoformat using clang and black
declare -a gitmodules
declare -a gitignores
gitignore=($(cat ../.gitignore |  grep /ros2/src/ | grep -v "!"| sed 's/rover\/ros2\/src\///g')) || gitignore=()
gitmodules=($(cat ../.gitmodules |  grep path | sed 's/	path = //' | sed 's/rover\/ros2\/src\///g')) || gitmodules=()
ignore="$(echo ${gitignore[@]} | sed 's/ /\\|/g')"
ignore+="\|"
modules="$(echo ${gitmodules[@]} | sed 's/ /\\|/g')"
modules+="\|ros2/build/*\|ros2/install/*"
find /workspace/rover/ros2/src -iname *.hpp -o -iname *.cpp -o -iname *.h -o -iname *.c | grep -v "Eigen" | grep -v $ignore$modules |  xargs clang-format -i

# Save actual folder and change directory to rover/
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
WORKSPACE="$SCRIPT_DIR/../"
cd $WORKSPACE/rover

modules="$(echo $modules | sed 's/\\//g')"

black --exclude="$modules" .
