cmake_minimum_required(VERSION 2.8)

find_package(Protobuf REQUIRED)

message( STATUS ${msgs} )
message( STATUS ${PROTO_SRCS} )
message( STATUS ${PROTOBUF_LIBRARY} )

set (msgs
  matrix.proto
)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library( matrix SHARED ${PROTO_SRCS})
target_link_libraries( matrix ${PROTOBUF_LIBRARY})
