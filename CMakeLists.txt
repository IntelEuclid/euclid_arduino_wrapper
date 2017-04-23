cmake_minimum_required(VERSION 2.8.3)
project(euclid_arduino_wrapper)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dynamic_reconfigure
  image_transport
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
 generate_dynamic_reconfigure_options(
   cfg/EuclidArduinoWrapper.cfg
   cfg/EuclidMotorConfiguration.cfg
 )


catkin_package(
  CATKIN_DEPENDS std_msgs dynamic_reconfigure
)
 
###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_library(EuclidArduinoWrapper src/euclid_arduino_wrapper.cpp)


add_library(EuclidArduinoMotorConfiguration src/euclid_arduino_motor_configuration.cpp
)

target_link_libraries( EuclidArduinoWrapper 
						${catkin_LIBRARIES} 
						${roscpp_LIBRARIES} 						
)

add_dependencies(EuclidArduinoWrapper ${PROJECT_NAME}_gencfg )

target_link_libraries( EuclidArduinoMotorConfiguration 
						${catkin_LIBRARIES} 
						${roscpp_LIBRARIES} 						
)

add_dependencies(EuclidArduinoMotorConfiguration ${PROJECT_NAME}_gencfg )

# Install nodelet library
install(TARGETS EuclidArduinoWrapper EuclidArduinoMotorConfiguration
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install xml files
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
