add_library(cuda_util INTERFACE)
target_link_libraries(cuda_util INTERFACE Eigen3::Eigen)
target_include_directories(cuda_util SYSTEM INTERFACE ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
