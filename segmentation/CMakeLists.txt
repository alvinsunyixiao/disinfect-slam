add_library(segmentation inference.cc)
<<<<<<< HEAD
=======
include_directories(${CMAKE_SOURCE_DIR})
>>>>>>> origin/master
target_link_libraries(segmentation PUBLIC ${TORCH_LIBRARIES} ${OpenCV_LIBS})