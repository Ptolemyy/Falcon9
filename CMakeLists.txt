cmake_minimum_required(VERSION 3.16)
project(falcon9_gui_planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(falcon9_gui_planner WIN32 falcon9_gui_planner.cpp)
target_compile_definitions(falcon9_gui_planner PRIVATE UNICODE _UNICODE)
target_link_libraries(falcon9_gui_planner PRIVATE user32 gdi32 comctl32 shell32)

add_executable(planner_comm_bridge planner_comm_bridge.cpp)
target_compile_definitions(planner_comm_bridge PRIVATE UNICODE _UNICODE)
