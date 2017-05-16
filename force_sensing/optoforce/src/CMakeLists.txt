
include_directories(../include)
add_library(liboptoforce MinimalOpto.cpp)

add_definitions(-std=c++0x)

add_executable(TEST__liboptoforce optotester.cpp)
target_link_libraries(TEST__liboptoforce liboptoforce pthread)