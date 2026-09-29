cmake_minimum_required(VERSION 3.18)

project(AmyCore VERSION 0.1.0)   # AmyCore's version number (build_info.h)

set(CMAKE_CXX_STANDARD 20)  

# Config files, meshes and runtime state are found relative to the source tree (AmyConfig/, thirdparty/)
add_compile_definitions(AMY_ROOT="${CMAKE_SOURCE_DIR}")

# Dear ImGui (MIT): -DIMGUI_DIR=<checkout> uses a local copy, otherwise a pinned revision is fetched
set(IMGUI_DIR "" CACHE PATH "Local Dear ImGui checkout (empty = fetch)")
if (NOT IMGUI_DIR)
    include(FetchContent)
    FetchContent_Declare(imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG        88fec097158867edf7753fe0d842d23d67e7966d)   # v1.89.9+ (master, 2023-11), the revision AmyCore was developed with
    FetchContent_MakeAvailable(imgui)
    set(IMGUI_DIR ${imgui_SOURCE_DIR})
endif()

find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

add_library(imgui STATIC
        ${IMGUI_DIR}/imgui.cpp
        ${IMGUI_DIR}/imgui_draw.cpp
        ${IMGUI_DIR}/imgui_tables.cpp
        ${IMGUI_DIR}/imgui_widgets.cpp
        ${IMGUI_DIR}/imgui_demo.cpp
        # SDL2 + OpenGL3 backend
        ${IMGUI_DIR}/backends/imgui_impl_sdl2.cpp
        ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp)
target_include_directories(imgui PUBLIC ${IMGUI_DIR}/ ${IMGUI_DIR}/backends/)

# Pinocchio (robotpkg, /opt/openrobots) for the gravity model, in its own small library (gravity.h: plain arrays only,
# so the rest of AmyCore doesn't need Pinocchio's headers)
find_package(pinocchio REQUIRED PATHS /opt/openrobots)
add_library(amygravity STATIC gravity.cpp collision.cpp)
target_link_libraries(amygravity PRIVATE pinocchio::pinocchio pinocchio::pinocchio_collision)
target_compile_options(amygravity PRIVATE -O2)   # Pinocchio is header templates: unoptimised, a collision check takes ~30 ms

add_executable(AmyCore udp_sender.cpp sawyerFrames.cpp amyEth.cpp linkdiag.cpp)
target_sources(AmyCore PRIVATE main.cpp )

# build_info.cpp: version, git revision and build time. Regenerated only when a source file or the git HEAD changes,
# so a no-op build doesn't relink AmyCore (relinking drops the setcap capabilities).
file(GLOB AMY_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/*.cpp ${CMAKE_SOURCE_DIR}/*.h)
set(AMY_BUILD_INFO_DEPS ${AMY_SOURCES} ${CMAKE_SOURCE_DIR}/cmake/build_info.cmake)
if (EXISTS ${CMAKE_SOURCE_DIR}/.git/logs/HEAD)
    list(APPEND AMY_BUILD_INFO_DEPS ${CMAKE_SOURCE_DIR}/.git/logs/HEAD)   # changes on commit/checkout
endif()
add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/build_info.cpp
    COMMAND ${CMAKE_COMMAND} -DSRC_DIR=${CMAKE_SOURCE_DIR} -DOUT=${CMAKE_BINARY_DIR}/build_info.cpp
            -DVERSION=${PROJECT_VERSION} -P ${CMAKE_SOURCE_DIR}/cmake/build_info.cmake
    DEPENDS ${AMY_BUILD_INFO_DEPS}
    COMMENT "Generating build_info.cpp")
target_sources(AmyCore PRIVATE ${CMAKE_BINARY_DIR}/build_info.cpp)
target_include_directories(AmyCore PRIVATE ${CMAKE_SOURCE_DIR})
# AmyCore runs with file capabilities (setcap), which makes the loader ignore LD_LIBRARY_PATH: find Pinocchio by RPATH
set_target_properties(AmyCore PROPERTIES BUILD_RPATH /opt/openrobots/lib INSTALL_RPATH /opt/openrobots/lib)

target_link_libraries(AmyCore GL ${SDL2_LIBRARIES} imgui amygravity)

# Relinking drops the file capabilities AmyCore needs (raw Ethernet socket, real-time priority, mlockall). Re-apply them
# after every link through the one sudo rule tools/install-setcap-sudoers.sh installs (once per machine); without that
# rule, or in another build dir, the build still succeeds and only warns. AmyCore itself refuses to start without them.
# Only for the real build dir (build-robot): a scratch build must never get the rights to talk to the robot.
set(AMY_CAPS cap_net_raw,cap_net_admin,cap_sys_nice,cap_ipc_lock+ep)
if (CMAKE_BINARY_DIR STREQUAL "${CMAKE_SOURCE_DIR}/build-robot")
add_custom_command(TARGET AmyCore POST_BUILD
    COMMAND sh -c "sudo -n /usr/sbin/setcap ${AMY_CAPS} '$<TARGET_FILE:AmyCore>' 2>/dev/null && echo 'AmyCore: capabilities set (${AMY_CAPS})' || echo 'WARNING: AmyCore capabilities NOT set; run tools/install-setcap-sudoers.sh once, or: sudo setcap ${AMY_CAPS} $<TARGET_FILE:AmyCore>'"
    VERBATIM)
endif()

