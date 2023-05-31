cmake_minimum_required(VERSION 3.16.3)



project(mavFlightManager LANGUAGES CXX)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

include(CheckCXXCompilerFlag)
include(CMakePrintHelpers)

if(NOT CMAKE_CXX_STANDARD) 
    set(CMAKE_CXX_STANDARD 17) 
endif()

find_package(MAVSDK REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fastrtps REQUIRED)
find_package(fastcdr REQUIRED)

add_subdirectory(external/yaml-cpp)
add_subdirectory(external/fmt)
add_subdirectory(external/sockpp)

set(MAVLINK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/mavlink/v2.0/common)
set(IDL_MSG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/msgs_idl)

file(GLOB DDS_TYPES_SOURCES_CXX 
    "${IDL_MSG_DIR}/builtin_interfaces/msg/*.cxx"
    "${IDL_MSG_DIR}/geometry_msgs/msg/*.cxx"
    "${IDL_MSG_DIR}/std_msgs/msg/*.cxx")

# persistent copy to common_parameters file during every compile
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config/common_parameters.yaml 
    ${CMAKE_CURRENT_BINARY_DIR}/config/common_parameters.yaml 
    COPYONLY
)

add_executable(dds_mav_test 
    src/test/dds_mav_pub.cpp 
    src/utils.cpp
    ${DDS_TYPES_SOURCES_CXX})
target_include_directories(dds_mav_test 
    PUBLIC 
    ${EIGEN3_INCLUDE_DIRS}
    ${MAVLINK_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${IDL_MSG_DIR})
target_link_libraries(dds_mav_test 
    fastrtps 
    fastcdr 
    fastdds::optionparser
    MAVSDK::mavsdk
    fmt::fmt)

add_executable(flight_manager 
    src/flightManager.cpp
    ${DDS_TYPES_SOURCES_CXX})
target_include_directories(flight_manager 
    PUBLIC 
    ${EIGEN3_INCLUDE_DIRS}
    ${MAVLINK_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${IDL_MSG_DIR})
target_link_libraries(flight_manager 
    fastrtps 
    fastcdr 
    fastdds::optionparser
    yaml-cpp::yaml-cpp
    MAVSDK::mavsdk
    fmt::fmt)

add_executable(launcher src/launcher.cpp)
target_link_libraries(launcher fmt::fmt)

# set(SOCKPP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/sockpp/include)

# add_executable(flightManager
#     src/flightManager.cpp
#     src/utils.cpp
# )
# target_include_directories(flightManager 
#     PUBLIC
#     ${CMAKE_CURRENT_SOURCE_DIR}/include
#     ${EIGEN3_INCLUDE_DIRS}
# )
# target_link_libraries(flightManager
#     MAVSDK::mavsdk
#     yaml-cpp::yaml-cpp
#     fmt::fmt
# )

# # mavlink proxy sender
# add_executable(mavlinkProxy
#     src/mavAgentProxy.cpp
#     src/utils.cpp
# )
# target_include_directories(mavlinkProxy 
#     PUBLIC
#     ${CMAKE_CURRENT_SOURCE_DIR}/include
#     ${EIGEN3_INCLUDE_DIRS}
#     ${MAVLINK_INCLUDE_DIR}
#     ${SOCKPP_INCLUDE_DIR}
    
#     PRIVATE
#     ${CMAKE_CURRENT_BINARY_DIR}/external/generated
# )
# set_target_properties(mavlinkProxy PROPERTIES
#     CXX_EXTENSIONS OFF
# )
# target_link_libraries(mavlinkProxy
#     MAVSDK::mavsdk
#     yaml-cpp::yaml-cpp
#     fmt::fmt
#     sockpp
# )
# target_compile_options(mavlinkProxy 
#     PRIVATE -DCMAKE_CXX_FLAGS="-Werror -Wall -Wextra -Wno-dev")