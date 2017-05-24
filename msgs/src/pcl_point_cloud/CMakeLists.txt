cmake_minimum_required(VERSION 2.8)
find_package(Protobuf REQUIRED)

#set(PROTOBUF_IMPORT_DIRS "/usr/local/include/gazebo-2.0/gazebo/msgs/proto")

#message( STATUS "TEST" )
#message( STATUS "PROTOBUF_IMPORT_DIRS: "${PROTOBUF_IMPORT_DIRS} )

message( STATUS ${msgs} )
message( STATUS ${PROTO_SRCS} )
message( STATUS ${PROTOBUF_LIBRARY} )

set (msgs
  point_cloud.proto
  point_type.proto
)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library( point_cloud SHARED ${PROTO_SRCS})
add_library( point_type SHARED ${PROTO_SRCS})
target_link_libraries( point_cloud point_type ${PROTOBUF_LIBRARY})
