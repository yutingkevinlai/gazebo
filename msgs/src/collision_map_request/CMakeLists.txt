cmake_minimum_required(VERSION 3.5)

find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS "/usr/local/include/gazebo/msgs/proto")

message( STATUS "TEST" )
message( STATUS "PROTOBUF_IMPORT_DIRS: " ${PROTOBUF_IMPORT_DIRS} )

set (msgs collision_map_request.proto ${PROTOBUF_IMPORT_DIRS}/vector2d.proto ${PROTOBUF_IMPORT_DIRS}/header.proto ${PROTOBUF_IMPORT_DIRS}/time.proto)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})

add_library(collision_map_creator_msgs SHARED ${PROTO_SRCS})
target_link_libraries(collision_map_creator_msgs ${PROTOBUF_LIBRARY})
