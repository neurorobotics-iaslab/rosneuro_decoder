cmake_minimum_required(VERSION 3.0.2)
project(rosneuro_decoder)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(ROSNEURO_DATA_MIN_VERSION "1.0.0")

##############################################################################
# Find dependencies
##############################################################################

find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 std_msgs
			 pluginlib
			 rosneuro_msgs)

find_package(Eigen3 REQUIRED)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)


##############################################################################
# Define package
##############################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  ${PROJECT_NAME}
	
  CATKIN_DEPENDS
  	roscpp
	std_msgs
	pluginlib
	rosneuro_msgs
  DEPENDS
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} 
                    ${EIGEN3_INCLUDE_DIRS}
					include)

add_definitions(${EIGEN3_DEFINITIONS})

add_library(${PROJECT_NAME} src/GenericDecoder.cpp
							src/Decoder.cpp)

target_link_libraries(${PROJECT_NAME} ${Eigen3_LIBRARIES} 
									  ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} 
				 ${catkin_EXPORTED_TARGETS}
)

add_executable(decoder src/decoder.cpp)
target_link_libraries(decoder ${PROJECT_NAME} ${catkin_LIBRARIES}) 

