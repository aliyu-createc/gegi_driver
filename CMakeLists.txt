cmake_minimum_required(VERSION 3.5)
project(phds_gegi_driver)

# CMAKE PARAMETERS
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
		roscpp
		rospy
		std_msgs
		nodelet
		pluginlib
		radiation_detector_msgs
		)

add_service_files(
  FILES
  StartAcquisition.srv
  StopAcquisition.srv
  ClearData.srv
  GetRunInfo.srv
  GetDetectorInfo.srv
  ToggleBiasMode.srv
  StartTimedAcquisition.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
                 INCLUDE_DIRS include
            CATKIN_DEPENDS radiation_detector_msgs std_msgs message_runtime
              )

find_package(Boost REQUIRED system thread regex)

# INCLUDES
include_directories(
		include
		${catkin_INCLUDE_DIRS}
		${Boost_INCLUDE_DIRS}
)

# LIBRARIES
add_library(tcp_event_reader src/phds_gegi_driver/socket_comms/tcp_event_reader.cpp)
target_link_libraries(tcp_event_reader ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_REGEX_LIBRARY})

add_library(${PROJECT_NAME} src/phds_gegi_driver/ros_phds_gegi_driver/phds_gegi_driver.cpp)
target_link_libraries(${PROJECT_NAME} tcp_event_reader ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} radiation_detector_msgs_generate_messages_cpp ${PROJECT_NAME}_generate_messages_cpp)

add_library(${PROJECT_NAME}_nodelet src/phds_gegi_driver/ros_phds_gegi_driver/phds_gegi_driver_nodelet.cpp )
target_link_libraries(${PROJECT_NAME}_nodelet ${PROJECT_NAME} ${catkin_LIBRARIES})

## Executables
add_executable(${PROJECT_NAME}_node src/phds_gegi_driver/ros_phds_gegi_driver/phds_gegi_driver_node.cpp )
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})

## Python nodes
catkin_install_python(PROGRAMS
  src/phds_gegi_driver/spectrum_node.py
  src/phds_gegi_driver/spherical_heatmap_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

