cmake_minimum_required(VERSION 3.13)
project(gfold_cpg_solver LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(GFOLD_SOLVER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/GFOLD_solver")

function(add_generated_backend target_name backend_cpp generated_c_dir)
    if(NOT EXISTS "${generated_c_dir}/include/cpg_workspace.h")
        message(FATAL_ERROR "Missing generated solver headers: ${generated_c_dir}")
    endif()

    set(_cpg_src
        "${generated_c_dir}/src/cpg_workspace.c"
        "${generated_c_dir}/src/cpg_solve.c"
    )

    file(GLOB _ecos_src "${generated_c_dir}/solver_code/src/*.c")
    list(REMOVE_ITEM _ecos_src
        "${generated_c_dir}/solver_code/src/runecos.c"
        "${generated_c_dir}/solver_code/src/runecos_exp.c"
    )
    file(GLOB _ecos_bb_src "${generated_c_dir}/solver_code/ecos_bb/*.c")
    file(GLOB _amd_src "${generated_c_dir}/solver_code/external/amd/src/*.c")
    file(GLOB _ldl_src "${generated_c_dir}/solver_code/external/ldl/src/*.c")

    add_library(${target_name} SHARED
        "${backend_cpp}"
        ${_cpg_src}
        ${_ecos_src}
        ${_ecos_bb_src}
        ${_amd_src}
        ${_ldl_src}
    )

    target_include_directories(${target_name} PRIVATE
        "${GFOLD_SOLVER_DIR}"
        "${generated_c_dir}/include"
        "${generated_c_dir}/solver_code/include"
        "${generated_c_dir}/solver_code/external/SuiteSparse_config"
        "${generated_c_dir}/solver_code/external/amd/include"
        "${generated_c_dir}/solver_code/external/ldl/include"
    )

    # Keep index width consistent with CVXPYgen-generated cpg_* code (cpg_int = int).
    # Forcing DLONG/LDL_LONG makes idxint=long long in ecos.h and can corrupt calls.
    target_compile_definitions(${target_name} PRIVATE CTRLC=1)
    target_compile_features(${target_name} PRIVATE cxx_std_17)
    set_target_properties(${target_name} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
endfunction()

add_generated_backend(
    gfold_solver_backend_n3
    "${GFOLD_SOLVER_DIR}/GFOLD_solver_3.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/p3/c"
)
add_generated_backend(
    gfold_solver_backend_n10
    "${GFOLD_SOLVER_DIR}/GFOLD_solver_10.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/p4_n10/c"
)
add_generated_backend(
    gfold_solver_backend_n25
    "${GFOLD_SOLVER_DIR}/GFOLD_solver_25.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/p4_n25/c"
)
add_generated_backend(
    gfold_solver_backend_n50
    "${GFOLD_SOLVER_DIR}/GFOLD_solver_50.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/p4_n50/c"
)
add_generated_backend(
    gfold_solver_backend_n100
    "${GFOLD_SOLVER_DIR}/GFOLD_solver_100.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/p4_n100/c"
)

add_library(gfold_solver_router STATIC
    "${GFOLD_SOLVER_DIR}/GFOLD_solver.cpp"
    "${GFOLD_SOLVER_DIR}/GFOLD_solver.hpp"
    "${GFOLD_SOLVER_DIR}/GFOLD_solver_backend_api.hpp"
)

target_include_directories(gfold_solver_router PUBLIC
    "${GFOLD_SOLVER_DIR}"
)

target_link_libraries(gfold_solver_router PUBLIC
    gfold_solver_backend_n3
    gfold_solver_backend_n10
    gfold_solver_backend_n25
    gfold_solver_backend_n50
    gfold_solver_backend_n100
)

add_library(gfold_solver ALIAS gfold_solver_router)
