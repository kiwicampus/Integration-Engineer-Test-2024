CURRENT_FOLDER=$PWD

if [ -f "/workspace/scripts/auto_format.sh" ]
then
  cd "/workspace/scripts"
  bash auto_format.sh
fi

cd /workspace/rover/ros2
colcon build "$@" || true
if [ -f "install/setup.bash" ]
then
    source install/setup.bash
fi
cd $CURRENT_FOLDER