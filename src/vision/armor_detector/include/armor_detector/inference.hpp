/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-21 16:24:35
 * @LastEditTime: 2023-03-17 18:53:13
 * @FilePath:
 * /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/include/inference/inference_api2.hpp
 */
#ifndef INFERENCE_API2_HPP_
#define INFERENCE_API2_HPP_

// c++
#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

// openvino
#include <openvino/openvino.hpp>

// opencv
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

// eigen
#include <Eigen/Core>

namespace armor_detector {
struct Object {
    cv::Rect_<float> rect;
    int cls;
    int color;
    float prob;
    std::vector<cv::Point2f> pts;
};
struct GridAndStride {
    int grid0;
    int grid1;
    int stride;
};

struct ArmorObject : Object {
    int area;
    cv::Point2f apex[4];
};

class Inference {
public:
    Inference();
    ~Inference();
    bool detect(const cv::Mat& src, std::vector<ArmorObject>& objects);
    bool initModel(const std::string& path);

    const std::vector<std::string> labels_lookup = {
        "",
        "1",
        "2",
        "3",
        "4",
        "5",
        "outpost",
        "sentry",
        "base",
    };

private:
    // int dw, dh;
    // float rescale_ratio;

    ov::Core core;
    std::shared_ptr<ov::Model> model; // 网络
    ov::CompiledModel compiled_model; // 可执行网络
    ov::InferRequest infer_request;   // 推理请求
    ov::Tensor input_tensor;

    std::string input_name;
    std::string output_name;

    Eigen::Matrix<float, 3, 3> transfrom_matrix;
};

} // namespace armor_detector

#endif