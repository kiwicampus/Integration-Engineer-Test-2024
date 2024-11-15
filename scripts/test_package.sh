packages=$@
current_dir=$PWD

export CYCLONEDDS_URI=/workspace/rover/configs/cyclone_dev_configuration.xml

cd /workspace/rover/ros2
source install/setup.bash

colcon test --packages-select $packages
colcon test-result --all --verbose
cd $current_dir