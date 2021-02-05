add_executable(l515_imu_async imu_async.cc)
target_link_libraries(l515_imu_async PRIVATE l515)

add_executable(l515_imu_pipe imu_pipe.cc)
target_link_libraries(l515_imu_pipe PRIVATE realsense2)

add_executable(l515_imshow_async imshow_async.cc)
target_link_libraries(l515_imshow_async PRIVATE l515)

add_executable(l515_show_calibration show_calibration.cc)
target_link_libraries(l515_show_calibration PRIVATE l515)

add_executable(l515_capture capture.cc)
target_link_libraries(l515_capture PRIVATE l515 popl)

add_executable(rs-motion rs-motion.cc)
target_link_libraries(rs-motion PRIVATE l515 gl_utils)
