cmake_minimum_required(VERSION 2.8.3)
project(ros_dual2equi)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  image_transport
  cv_bridge
  image_view
  
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_dual2equi
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)


add_executable(dual2equi_fromtable src/dual2equi_fromtable.cpp)
target_link_libraries(dual2equi_fromtable ${catkin_LIBRARIES})
install(TARGETS dual2equi_fromtable
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


catkin_install_python(PROGRAMS 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/save_images_with_timestamps.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
