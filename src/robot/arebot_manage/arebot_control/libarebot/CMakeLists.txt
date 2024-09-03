cmake_minimum_required(VERSION 3.6)
project(libarebot)

set(CMAKE_CXX_STANDARD 14)

find_package(Threads REQUIRED)
find_package(Protobuf REQUIRED)

add_subdirectory(proto)

add_library(arebot STATIC src/BotControl.cpp include/BotControl.h include/BotControlCallbacks.h src/Serial.cpp include/Serial.h)
target_include_directories(arebot PUBLIC include)
target_link_libraries(arebot arebot_proto ${Protobuf_LIBRARIES})
set_property(TARGET arebot PROPERTY POSITION_INDEPENDENT_CODE ON)

add_executable(arebot_cli main.cpp)
target_link_libraries(arebot_cli arebot)