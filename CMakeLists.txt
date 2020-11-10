cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(
        GazeboVehicleSimBindings
        VERSION 1.0
)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

file(TO_NATIVE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/msgs PROTOMODEL_PATH)
file(TO_NATIVE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/msgs/include PROTOBINDING_PATH)

file(GLOB DATAMODEL_PROTOS "${CMAKE_CURRENT_SOURCE_DIR}/msgs/*.proto")

foreach(proto ${DATAMODEL_PROTOS})
    file(TO_NATIVE_PATH ${proto} proto_native)
    execute_process(COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} --proto_path=${PROTOMODEL_PATH} --cpp_out=${PROTOBINDING_PATH} ${proto_native}
            RESULT_VARIABLE rv)
    # Optional, but that can show the user if something have gone wrong with the proto generation
    if(${rv})
        message("Generation of data model returned ${rv} for proto ${proto_native}")
    endif()
endforeach(proto)

# Build our plugin
find_package(Eigen3)
file(GLOB PROTO_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/msgs/include/*.cc")

# two ways to add_library here, I can add my own compiled protobuf files or I can grab the already compiled ones from gazebo's source
# target_link_libraries(rotor_plugin ${GAZEBO_LIBRARIES} WindPlugin Eigen3::Eigen)

add_library(rotor_plugin SHARED src/rotor_plugin.cpp ${PROTO_SRCS})
target_link_libraries(rotor_plugin ${GAZEBO_LIBRARIES} Eigen3::Eigen)
target_include_directories(rotor_plugin PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Build the hover envelope control module
add_executable(hover_env_control src/hover_env_src/hover_env_control.cpp include/hover_env_include/hover_env_control.hpp ${PROTO_SRCS})
target_link_libraries(hover_env_control ${GAZEBO_LIBRARIES} Eigen3::Eigen)
#target_include_directories(set_rotor_vel PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/msgs/include/)

# Build the geometric control module
add_executable(geometric_control src/geometric_src/geometric_control.cpp include/geometric_include/geometric_control.h ${PROTO_SRCS})
target_link_libraries(geometric_control ${GAZEBO_LIBRARIES} Eigen3::Eigen)