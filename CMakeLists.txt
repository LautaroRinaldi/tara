cmake_minimum_required(VERSION 3.3)
project(uvc_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(rmw_implementation REQUIRED)


FIND_PACKAGE(PkgConfig)
PKG_CHECK_MODULES(PC_GLIB QUIET glib-2.0)

FIND_LIBRARY(
	GLIB_LIBRARIES
	NAMES glib-2.0
	HINTS ${PC_GLIB_LIBDIR}
	${PC_GLIB_LIBRARY_DIRS}
)

# Files in glib's main include path may include glibconfig.h, which,
# for some odd reason, is normally in $LIBDIR/glib-2.0/include.
GET_FILENAME_COMPONENT(_GLIB_LIBRARY_DIR ${GLIB_LIBRARIES} PATH)
FIND_PATH(
	GLIBCONFIG_INCLUDE_DIR
	NAMES glibconfig.h
	HINTS ${PC_LIBDIR} ${PC_LIBRARY_DIRS} ${_GLIB_LIBRARY_DIR}
	PATH_SUFFIXES glib-2.0/include
)

FIND_PATH(
	GLIB_INCLUDE_DIR
	NAMES glib.h
	HINTS ${PC_GLIB_INCLUDEDIR}
	${PC_GLIB_INCLUDE_DIRS}
	PATH_SUFFIXES glib-2.0
)

SET(GLIB_INCLUDE_DIRS ${GLIB_INCLUDE_DIR} ${GLIBCONFIG_INCLUDE_DIR})

include_directories(
	${rclcpp_INCLUDE_DIRS}
	${rmw_implementation_INCLUDE_DIRS}
	${std_msgs_INCLUDE_DIRS}
	${sensor_msgs_INCLUDE_DIRS}
	#[[las 3 de abajo estan hardcodeadas, mejorar]]
	/opt/ros/humble/include/camera_info_manager/
	/opt/ros/humble/include/image_transport/
	/opt/ros/humble/include/camera_calibration_parsers/
	/opt/ros/humble/include/pluginlib/
	#[[class_loader la usa pluginlib]]
	/opt/ros/humble/include/class_loader/
	${GLIB_INCLUDE_DIRS}
	include
	${CMAKE_SOURCE_DIR}/src/xunit
)
#[[compilo solo lo necesario para Tara
add_library(xunit
  src/xunit/xunit_lib.c
  src/xunit/xunit_lib_cug.c
)
]]

add_library(xunitTara
  src/xunit/xunit_lib_tara.h
  src/xunit/xunit_lib_tara.cpp
) 

target_link_libraries(xunitTara ${udev_LIBRARIES} ${GLIB2_LIBRARIES})  

#[[compilo solo lo necesario para Tara
target_link_libraries(xunit ${udev_LIBRARIES})

add_library(nodelet_uvc_camera src/nodelets.cpp src/camera.cpp src/stereo.cpp src/uvc_cam.cpp)
target_link_libraries(nodelet_uvc_camera
	${rclcpp_LIBRARIES}
	${rmw_implementation_LIBRARIES}
	${std_msgs_LIBRARIES}
	xunit
)

add_executable(uvc_camera_node src/camera_node.cpp src/camera.cpp src/uvc_cam.cpp)
target_link_libraries(uvc_camera_node
	${rclcpp_LIBRARIES}
	${rmw_implementation_LIBRARIES}
	${std_msgs_LIBRARIES}
	xunit
	xunitTara
	-lv4l2 -ludev -lglib-2.0
)

add_executable(uvc_stereo_node src/stereo_node.cpp src/stereo.cpp src/uvc_cam.cpp)
target_link_libraries(uvc_stereo_node
	${rclcpp_LIBRARIES}
	${camera_info_manager_LIBRARIES}
	${image_transport_LIBRARIES}
	${rmw_implementation_LIBRARIES}
	${std_msgs_LIBRARIES}
	xunit
	xunitTara
	-lv4l2 -ludev -lglib-2.0
)]]

add_executable(tara_node src/tara_node.cpp src/tara_ros.cpp src/uvc_cam.cpp)
target_link_libraries(tara_node
	${rclcpp_LIBRARIES}
	${camera_info_manager_LIBRARIES}
	${image_transport_LIBRARIES}
	${rmw_implementation_LIBRARIES}
	${std_msgs_LIBRARIES}
	xunitTara
	${GLIB2_LIBRARIES}
	-lv4l2 -lyaml-cpp -ludev -lglib-2.0
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION lib/${PROJECT_NAME}
# )

## Mark executables and/or libraries for installation
#[[compilo solo lo necesario para Tara
install(TARGETS nodelet_uvc_camera uvc_camera_node uvc_stereo_node
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION lib/${PROJECT_NAME}
)]]

install(TARGETS tara_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#[[compilo solo lo necesario para Tara
install(FILES
  nodelet_uvc_camera.xml
  DESTINATION share/${PROJECT_NAME}
)]]

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION share/${PROJECT_NAME}
# )

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rclcpp sensor_msgs camera_info_manager image_transport)
ament_export_include_directories(include)
ament_export_libraries(nodelet_uvc_camera)
#ament_export_dependencies(system_lib)
ament_package()

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_uvc_camera.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
