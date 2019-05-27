## build in docker

cd scripts/

bash build.sh 

## run four nodes in order in four terminators both in docker

bash lidar_registor.sh

bash lidar_odo.sh

bash lidar_mapping.sh

bash display.sh

## paly rosbag

rosbag play *.bag  (in your path)