cmake_minimum_required(VERSION 3.16)
project(falcon9_gui_planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(
    falcon9_gui_planner
    WIN32
    falcon9_gui_planner.cpp
    planner_mission.cpp
    planner_upfg.cpp
    planner_recovery.cpp)
target_compile_definitions(falcon9_gui_planner PRIVATE UNICODE _UNICODE)
target_link_libraries(falcon9_gui_planner PRIVATE user32 gdi32 comctl32 shell32)
set(FALCON9_DEFAULT_CONFIG "${CMAKE_SOURCE_DIR}/config/falcon9_real_defaults.txt")
add_custom_command(
    TARGET falcon9_gui_planner
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${FALCON9_DEFAULT_CONFIG}"
            "$<TARGET_FILE_DIR:falcon9_gui_planner>/falcon9_real_defaults.txt")

add_executable(planner_comm_bridge planner_comm_bridge.cpp)
target_compile_definitions(planner_comm_bridge PRIVATE UNICODE _UNICODE)

add_executable(
    planner_smoke
    planner_smoke.cpp
    planner_mission.cpp
    planner_upfg.cpp
    planner_recovery.cpp)
