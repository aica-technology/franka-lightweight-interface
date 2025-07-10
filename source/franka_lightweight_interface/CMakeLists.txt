cmake_minimum_required(VERSION 3.9)
project(franka_lightweight_interface VERSION 3.1)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  	set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  	set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(control_libraries 9.2.0 REQUIRED COMPONENTS state_representation communication_interfaces)
find_package(clproto 9.2.0 REQUIRED)
find_package(Franka 0.15.0 REQUIRED)

include_directories(
	include
)

set(CORE_SOURCES
	src/FrankaLightWeightInterface.cpp
	src/main.cpp
)

add_executable(${PROJECT_NAME} ${CORE_SOURCES})
target_link_libraries(${PROJECT_NAME} franka ${control_libraries_LIBRARIES} clproto)

install(TARGETS ${PROJECT_NAME}	RUNTIME DESTINATION bin)

add_executable(example_joint_position_controller examples/joint_position_controller.cpp)
target_link_libraries(example_joint_position_controller ${control_libraries_LIBRARIES} clproto)

add_executable(example_joint_velocity_controller examples/joint_velocity_controller.cpp)
target_link_libraries(example_joint_velocity_controller ${control_libraries_LIBRARIES} clproto)
