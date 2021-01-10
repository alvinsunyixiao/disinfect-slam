#include <iostream>
#include <vector>
#include <memory>
#include <chrono>
#include <cinttypes>
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>

#include "segmentation/inference.h"

int main() {
    inference_engine my_engine("/home/roger/disinfect-slam/segmentation/ht_lt.pt");
    // Load image for test run
    cv::Mat image_rgb, image_bgr;
    image_bgr = cv::imread("/home/roger/hospital_images/24.jpg");
    cv::cvtColor(image_bgr, image_rgb, cv::COLOR_BGR2RGB);

    my_engine.infer_one(image_rgb);
/*
    std::cout << "Input shape: " << my_gpu_tensor.sizes() << std::endl;
    const auto start = std::chrono::steady_clock::now();
    
    const auto now = std::chrono::steady_clock::now();
    auto time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
    std::cout << "Time elapsed (in milliseconds): " << time_elapsed << std::endl;
    std::cout << "Test image feeded." << std::endl;
    std::cout << "Output shape: " << test_image_output.sizes() << std::endl;
    // std::cout << test_image_output << std::endl;

    int num_trials = 1000;
    torch::Tensor loop_img_output;
    const auto loop_start = std::chrono::steady_clock::now();
    for (int i = 0; i < num_trials; ++i) {
        inputs.clear();
        inputs.push_back(torch::ones({1, 3, 352, 640}).to(torch::kCUDA));
        loop_img_output = module.forward(inputs).toTensor();
    }
    const auto loop_end = std::chrono::steady_clock::now();
    auto loop_total = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start).count();
    std::cout << "Loop total time (in milliseconds): " << loop_total << std::endl;
    std::cout << "Inference time per image (in milliseconds): " << ((uint32_t)loop_total / num_trials) << std::endl;

    test_image_output.to(torch::kCPU);
    cv::Mat ret = tensor_to_mat(test_image_output);
    */
    return 0;
}