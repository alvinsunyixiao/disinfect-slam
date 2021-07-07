add_library(popl INTERFACE)
target_include_directories(popl SYSTEM INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/popl/include)

add_subdirectory(imgui-1.77)
add_subdirectory(googletest-1.10.0)
add_subdirectory(ORB_SLAM3)

# NOTE(alvin): add back when OpenVSLAM is removed
# add_subdirectory(spdlog-1.6.1)
