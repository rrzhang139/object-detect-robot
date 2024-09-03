set(PROTO_SRCS BotRequest.proto BotResponse.proto)
protobuf_generate_cpp(PROTO_GEN_SRCS PROTO_HDRS ${PROTO_SRCS})

add_library(arebot_proto STATIC ${PROTO_GEN_SRCS} ${PROTO_HDRS})

target_include_directories(arebot_proto
        PRIVATE ${Protobuf_INCLUDE_DIRS}
        PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(arebot_proto ${Protobuf_LIBRARIES})
set_property(TARGET arebot_proto PROPERTY POSITION_INDEPENDENT_CODE ON)