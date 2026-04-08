cmake_minimum_required(VERSION 3.13)

set(_variant_name "p4_n100")
set(_generated_dir "${CMAKE_CURRENT_LIST_DIR}/../../p4_n100/c")

if(NOT EXISTS "${_generated_dir}/CMakeLists.txt")
    message(
        FATAL_ERROR
        "Generated solver not found for ${_variant_name}: ${_generated_dir}. "
        "Run: python solver_py/solver.py --gen"
    )
endif()

add_subdirectory("${_generated_dir}" "${CMAKE_CURRENT_BINARY_DIR}/${_variant_name}_generated")

set(_target_name "gfold_solver_${_variant_name}")
add_library(${_target_name} STATIC
    "${CMAKE_CURRENT_LIST_DIR}/../../GFOLD_solver/GFOLD_solver.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../GFOLD_solver/GFOLD_solver.hpp"
)

target_include_directories(${_target_name} PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/../../GFOLD_solver"
    "${_generated_dir}/include"
    "${_generated_dir}/solver_code"
    ${cpg_include}
)

target_link_libraries(${_target_name} PUBLIC cpg)
target_compile_features(${_target_name} PUBLIC cxx_std_17)

set(GFOLD_SOLVER_TARGET ${_target_name} PARENT_SCOPE)

