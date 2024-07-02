cmake_minimum_required(VERSION 3.0.2)
project(kinco_client)

add_compile_options(-std=c++2a)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS graph)
find_package(PkgConfig REQUIRED)

catkin_package(
   INCLUDE_DIRS include/kinco_client
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp  geometry_msgs
   )

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

set(PARAMS_PATH "${CMAKE_CURRENT_SOURCE_DIR}/params/")
add_definitions(-DPARAMS_PATH="${PARAMS_PATH}")

add_library(drivers 
            src/drivers/KincoMotor.cpp
            src/drivers/DifferentialDrive.cpp
            src/drivers/external/client.cpp
            src/drivers/external/common.cpp
            src/drivers/external/pipe_ret_t.cpp
            src/drivers/external/tcp_client.cpp
            src/drivers/external/tcp_server.cpp
            )

target_include_directories(drivers PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/kinco_client
    ${CMAKE_CURRENT_SOURCE_DIR}/include/kinco_client/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/include/kinco_client/drivers/external)

add_executable(${PROJECT_NAME} src/mainWrapper.cpp src/DeviceManager.cpp)

target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} drivers -lpthread)

install(
  DIRECTORY include
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
