# compile
mkdir -p xx/pcl_ws/src
cd xx/pcl_ws
catkin_make

# run
#one terminal
roscore
#another terminal
cd xx/pcl_ws
source devel/setup.bash
rosrun pcl_icp_test pcl_icp_test
