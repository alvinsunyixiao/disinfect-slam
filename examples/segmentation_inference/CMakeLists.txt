if (CMAKE_CUDA_COMPILER)
    add_executable(sample_inference main.cc)
<<<<<<< HEAD
    target_link_libraries(sample_inference PRIVATE segmentation ${OpenCV_LIBS})
=======
    target_link_libraries(sample_inference
        PRIVATE segmentation ${OpenCV_LIBS})
>>>>>>> origin/master
endif ()
