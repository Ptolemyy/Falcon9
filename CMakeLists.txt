cmake_minimum_required(VERSION 3.16)
project(falcon9_gui_planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(GFOLD/cpg_solver)

set(FALCON9_GFOLD_BACKEND_DLL_TARGETS
    gfold_solver_backend_n3
    gfold_solver_backend_n10
    gfold_solver_backend_n25
    gfold_solver_backend_n50
    gfold_solver_backend_n100
    gfold_solver_backend_2d_n50
)

function(copy_gfold_backend_runtime_deps exe_target)
    foreach(_dll_target IN LISTS FALCON9_GFOLD_BACKEND_DLL_TARGETS)
        add_custom_command(
            TARGET ${exe_target}
            POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "$<TARGET_FILE:${_dll_target}>"
                    "$<TARGET_FILE_DIR:${exe_target}>"
            VERBATIM
        )
    endforeach()
endfunction()

add_executable(
    falcon9_gui_planner
    WIN32
    falcon9_gui_planner.cpp
    planner_mission.cpp
    planner_lvd.cpp
    planner_upfg.cpp
    planner_recovery.cpp)
target_compile_definitions(falcon9_gui_planner PRIVATE UNICODE _UNICODE)
target_link_libraries(falcon9_gui_planner PRIVATE gfold_solver user32 gdi32 comctl32 shell32)
copy_gfold_backend_runtime_deps(falcon9_gui_planner)
set(FALCON9_DEFAULT_CONFIG "${CMAKE_SOURCE_DIR}/config/falcon9_real_defaults.txt")
set(FALCON9_EARTH_TEXTURE "${CMAKE_SOURCE_DIR}/assets/earth_blue_marble.bmp")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/assets")
configure_file("${FALCON9_DEFAULT_CONFIG}" "${CMAKE_BINARY_DIR}/falcon9_real_defaults.txt" COPYONLY)
configure_file("${FALCON9_EARTH_TEXTURE}" "${CMAKE_BINARY_DIR}/assets/earth_blue_marble.bmp" COPYONLY)

add_executable(planner_comm_bridge planner_comm_bridge.cpp)
target_compile_definitions(planner_comm_bridge PRIVATE UNICODE _UNICODE)

add_executable(
    planner_smoke
    planner_smoke.cpp
    planner_mission.cpp
    planner_lvd.cpp
    planner_upfg.cpp
    planner_recovery.cpp)
target_link_libraries(planner_smoke PRIVATE gfold_solver)
copy_gfold_backend_runtime_deps(planner_smoke)
