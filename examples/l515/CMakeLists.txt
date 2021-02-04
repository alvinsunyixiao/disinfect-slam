add_executable(l515_imu_async imu_async.cc)
target_link_libraries(l515_imu_async PRIVATE realsense2 l515)

add_executable(l515_imu_pipe imu_pipe.cc)
target_link_libraries(l515_imu_pipe PRIVATE realsense2)

add_executable(l515_imshow_async imshow_async.cc)
target_link_libraries(l515_imshow_async PRIVATE realsense2 l515 ${OpenCV_LIBS})
