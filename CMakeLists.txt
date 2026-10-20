cmake_minimum_required(VERSION 3.24)
project(customDragline LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(FATAL_ERROR "PhysX binaries are x64 only - configure with -A x64")
endif()

# ---------------------------------------------------------------------------
# raylib (fetched)
# ---------------------------------------------------------------------------
include(FetchContent)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES    OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG        5.5
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(raylib)

# ---------------------------------------------------------------------------
# Eigen (header-only, fetched; SOURCE_SUBDIR skips Eigen's own CMake project)
# ---------------------------------------------------------------------------
FetchContent_Declare(
    eigen
    URL           https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    URL_HASH      SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
    SOURCE_SUBDIR do-not-build
)
FetchContent_MakeAvailable(eigen)
add_library(Eigen INTERFACE)
target_include_directories(Eigen SYSTEM INTERFACE "${eigen_SOURCE_DIR}")

# ---------------------------------------------------------------------------
# PhysX 4.1.2 (prebuilt, win.x86_64.vc142.md -> dynamic CRT /MD)
# ---------------------------------------------------------------------------
set(PHYSX_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/PhysX-4.1.2" CACHE PATH "PhysX SDK root")
set(PHYSX_BIN  "${PHYSX_ROOT}/physx/bin/win.x86_64.vc142.md")

add_library(PhysX INTERFACE)
target_include_directories(PhysX INTERFACE
    "${PHYSX_ROOT}/physx/include"
    "${PHYSX_ROOT}/pxshared/include"
)

# Debug -> debug libs, everything else -> release libs
set(PHYSX_CFG "$<IF:$<CONFIG:Debug>,debug,release>")
set(PHYSX_LIBS
    PhysX_64
    PhysXCommon_64
    PhysXFoundation_64
    PhysXExtensions_static_64
    PhysXPvdSDK_static_64
)
foreach(lib IN LISTS PHYSX_LIBS)
    target_link_libraries(PhysX INTERFACE "${PHYSX_BIN}/${PHYSX_CFG}/${lib}.lib")
endforeach()

set(PHYSX_DLLS PhysX_64 PhysXCommon_64 PhysXFoundation_64)

# ---------------------------------------------------------------------------
# App
# ---------------------------------------------------------------------------
add_executable(customDragline
    src/main.cpp
    src/Dragline.cpp
    src/RigSolver.cpp
    src/Renderer.cpp
)
target_link_libraries(customDragline PRIVATE raylib PhysX Eigen)

if(MSVC)
    target_compile_options(customDragline PRIVATE /W4 /permissive-)
    set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT customDragline)
endif()

# Headless rig check (no window): build and run `dragline_selftest [ff|lower]`
add_executable(dragline_selftest
    src/selftest.cpp
    src/Dragline.cpp
    src/RigSolver.cpp
)
target_link_libraries(dragline_selftest PRIVATE PhysX Eigen)
if(MSVC)
    target_compile_options(dragline_selftest PRIVATE /W4 /permissive-)
endif()

foreach(target IN ITEMS customDragline dragline_selftest)
    foreach(dll IN LISTS PHYSX_DLLS)
        add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${PHYSX_BIN}/${PHYSX_CFG}/${dll}.dll"
                "$<TARGET_FILE_DIR:${target}>"
        )
    endforeach()
endforeach()
