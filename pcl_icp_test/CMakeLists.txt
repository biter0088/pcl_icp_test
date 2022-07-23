cmake_minimum_required(VERSION 3.0.2)
project(pcl_icp_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_conversions
)

set(PCL_DIR "/usr/share/pcl-1.8")#设置pcl库目录
find_package(PCL REQUIRED)


include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS   roscpp rospy std_msgs pcl_conversions
  DEPENDS  PCL 
  INCLUDE_DIRS include
)

# 单独调试pcl_icp
add_executable(pcl_icp_test src/pcl_icp_test.cpp)
target_link_libraries(pcl_icp_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})