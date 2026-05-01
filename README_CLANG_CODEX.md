# Clang / ClangCL + Codex Build Guide

This file is a reusable guide for asking Codex to configure and build C/C++ projects with Clang.

It is written for a Windows + CMake + VS Code workflow first, because that is the setup that caused the least friction in this project.

## Goal

When using Codex in another project, the preferred build path is:

- CMake Presets
- Visual Studio 2022 generator
- `ClangCL` toolset
- `x64` architecture
- separate build directory such as `build-clangcl`

This avoids common problems:

- accidentally using the default MSVC frontend instead of Clang
- mixing `Win32` and 64-bit Python
- generating only `Release` and then trying to build `Debug`
- stale cache problems inside an old `build/` directory

## Recommended Rules For Codex

If the project is on Windows and uses CMake:

1. Prefer `ClangCL` with `x64`.
2. Prefer `CMakePresets.json` over ad-hoc VS Code kit settings.
3. Use a dedicated build directory such as `build-clangcl`.
4. Do not force a Visual Studio multi-config project into a single `Release`-only configuration.
5. If the project uses Python or pybind11, make sure Python architecture matches the compiler architecture.
6. If an old `build/` cache exists and conflicts with the intended toolchain, create a fresh build directory instead of reusing the stale one.

## Minimal Windows Preset Template

Create a `CMakePresets.json` like this and adjust paths as needed:

```json
{
  "version": 6,
  "cmakeMinimumRequired": {
    "major": 3,
    "minor": 23,
    "patch": 0
  },
  "configurePresets": [
    {
      "name": "clangcl-x64",
      "displayName": "ClangCL x64",
      "generator": "Visual Studio 17 2022",
      "binaryDir": "${sourceDir}/build-clangcl",
      "architecture": "x64",
      "toolset": "ClangCL,host=x64",
      "cacheVariables": {
        "CMAKE_EXPORT_COMPILE_COMMANDS": "TRUE"
      }
    }
  ],
  "buildPresets": [
    {
      "name": "debug",
      "configurePreset": "clangcl-x64",
      "configuration": "Debug"
    },
    {
      "name": "release",
      "configurePreset": "clangcl-x64",
      "configuration": "Release"
    }
  ]
}
```

If the project depends on packages installed outside the repo, add them to `cacheVariables`, for example:

```json
"CMAKE_PREFIX_PATH": "C:/libs/pybind11;C:/libs/some_other_lib"
```

## Minimal VS Code Settings

Use this in `.vscode/settings.json`:

```json
{
  "C_Cpp.default.configurationProvider": "ms-vscode.cmake-tools",
  "cmake.useCMakePresets": "always"
}
```

This makes VS Code follow the preset workflow instead of drifting back to old kit-based configuration.

## CMakeLists Guidance

For Visual Studio generators, keep all normal configurations available:

```cmake
if(CMAKE_CONFIGURATION_TYPES)
    set(
        CMAKE_CONFIGURATION_TYPES
        "Debug;Release;RelWithDebInfo;MinSizeRel"
        CACHE STRING "Supported build configurations"
        FORCE
    )
elseif(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()
```

If the project must use Clang on Windows, add a guard like this:

```cmake
if(MSVC AND NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    message(FATAL_ERROR
        "This project must be configured with Visual Studio ClangCL x64."
    )
endif()
```

If the project uses pybind11, prefer the modern Python discovery path:

```cmake
set(PYBIND11_FINDPYTHON ON)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
```

## Standard Commands

Configure:

```powershell
cmake --preset clangcl-x64
```

Build Debug:

```powershell
cmake --build --preset debug
```

Build Release:

```powershell
cmake --build --preset release
```

## Common Failure Patterns

### 1. Python is 64-bit, chosen compiler is 32-bit

Cause:

- project was configured as `Win32` or `x86`
- Python install is 64-bit

Fix:

- use `x64`
- do not use `-A win32`
- do not use `host=x86`

### 2. Project does not contain Debug|x64

Cause:

- `CMakeLists.txt` forced `CMAKE_CONFIGURATION_TYPES` to only `Release`

Fix:

- restore the normal multi-config list
- keep `Debug` and `Release` available

### 3. CMake says Clang but generated project builds with MSVC frontend

Cause:

- old cache reused
- generator/toolset mismatch
- VS Code drifted back to a non-preset build path

Fix:

- use `CMakePresets.json`
- use a clean build directory such as `build-clangcl`
- set VS Code to `cmake.useCMakePresets = always`

### 4. Third-party headers compile with Clang but fail under MSVC

Cause:

- dependency contains GNU/Clang-only attributes or syntax

Fix:

- use `ClangCL`
- or add a small compatibility wrapper in the project instead of editing external libraries directly

## Copy-Paste Prompt For Codex

Use the following prompt in another project:

```text
Please configure and build this project with Clang on Windows.

Requirements:
- Use CMake Presets.
- Use Visual Studio 2022 generator.
- Use ClangCL toolset.
- Use x64 architecture.
- Use a dedicated build directory such as build-clangcl.
- Do not use Win32/x86.
- Do not force a Visual Studio multi-config project into Release-only.
- If Python/pybind11 is used, make sure Python architecture matches the compiler architecture.
- If the current build cache is stale or configured for a different toolchain, create a fresh build directory instead of reusing it.
- Prefer fixing the project so that `cmake --preset clangcl-x64` and `cmake --build --preset release` work cleanly.

Please inspect the current CMake setup first, then update CMakePresets.json, CMakeLists.txt, and VS Code settings if needed, and finally run a real configure + build to verify.
```

## Optional Non-Windows Note

For Linux or macOS projects, the equivalent idea is usually:

- generator: `Ninja`
- compiler: `clang` / `clang++`
- separate build directory such as `build-clang`

Typical commands:

```bash
cmake -S . -B build-clang -G Ninja -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++
cmake --build build-clang
```

## Summary

For Windows projects, the most repeatable setup is:

- `CMakePresets.json`
- `ClangCL`
- `x64`
- `build-clangcl`
- VS Code preset mode enabled

If Codex follows those rules, it is much less likely to end up in a broken MSVC/Win32/stale-cache configuration.
