cmake_minimum_required(VERSION 3.13)
project(gfold VERSION 1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_VERBOSE_MAKEFILE OFF)

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
if(CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_CONFIGURATION_TYPES "Release" CACHE STRING "" FORCE)
endif()


find_package(Python3 COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
find_package(matplotlibcpp17 REQUIRED)

add_subdirectory(cpg_solver)

add_executable(test_solver main.cpp)
add_executable(GFOLD_demo GFOLD_demo.cpp find_cfg.cpp plot_recv_mode0.cpp)
add_executable(kos_pingpong kos_pingpong.cpp)
add_executable(cfg_tool find_cfg_tool.cpp)

set(GFOLD_BACKEND_DLL_TARGETS
    gfold_solver_backend_n3
    gfold_solver_backend_n10
    gfold_solver_backend_n25
    gfold_solver_backend_n50
    gfold_solver_backend_n100
)

function(copy_backend_runtime_deps exe_target)
    foreach(_dll_target IN LISTS GFOLD_BACKEND_DLL_TARGETS)
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

target_include_directories(test_solver PUBLIC 
        ${Python3_INCLUDE_DIRS})

target_link_libraries(test_solver 
    gfold_solver
    ${Python3_LIBRARIES} 
    pybind11::embed
    matplotlibcpp17::matplotlibcpp17
)
copy_backend_runtime_deps(test_solver)

target_include_directories(GFOLD_demo PUBLIC 
        ${Python3_INCLUDE_DIRS})

target_link_libraries(GFOLD_demo 
    gfold_solver
    ${Python3_LIBRARIES}
    pybind11::embed
    matplotlibcpp17::matplotlibcpp17
)
copy_backend_runtime_deps(GFOLD_demo)

target_include_directories(cfg_tool PUBLIC
        ${Python3_INCLUDE_DIRS})

target_link_libraries(cfg_tool
    gfold_solver
    ${Python3_LIBRARIES}
    pybind11::embed
    matplotlibcpp17::matplotlibcpp17
)
copy_backend_runtime_deps(cfg_tool)
