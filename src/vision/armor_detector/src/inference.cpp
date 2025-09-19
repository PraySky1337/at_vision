/**
 * @projectName armor_auto_aim
 * @file inference.cpp
 * @brief
 *
 * @author yx
 * @date 2023-10-27 20:05
 */

#include "armor_detector/inference.hpp"

// ROS2
#include <rclcpp/logging.hpp>
// 3rdlibs
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>

#include "armor_detector/parser.hpp"

namespace armor_auto_aim {
Inference::Inference() {
    auto available = m_core.get_available_devices();
    RCLCPP_INFO_STREAM(
        rclcpp::get_logger("armor_detector"), fmt::format("Inference devices: {}", available));
    m_device =
        (std::find(available.begin(), available.end(), "GPU") != available.end()) ? "GPU" : "CPU";
    RCLCPP_INFO(rclcpp::get_logger("armor_detector"), "Inference device: %s", m_device.c_str());
}

Inference::Inference(const std::string& model_path)
    : m_MODEL_PATH(model_path)
    , m_inference_result_ptr(new float) {
    auto available = m_core.get_available_devices();
    RCLCPP_INFO_STREAM(
        rclcpp::get_logger("armor_detecotr"),
        fmt::format("Inference available devices: {}", available));
    m_device =
        (std::find(available.begin(), available.end(), "GPU") != available.end()) ? "GPU" : "CPU";
    RCLCPP_INFO(rclcpp::get_logger("armor_detecotr"), "Inference device: %s", m_device.c_str());
    initModel(m_MODEL_PATH);
}

void Inference::setLabelUrl(const std::string& label_path) {
    std::ifstream infile(label_path);
    if (!infile.is_open()) {
        std::cerr << "Failed to open label file: " << label_path << std::endl;
        return;
    }

    labels_lookup.clear();
    std::string line;
    while (std::getline(infile, line)) {
        if (!line.empty()) {
            labels_lookup.push_back(line);
        }
    }
    infile.close();
}

Inference::Inference(const std::string& model_path, const std::string& driver)
    : m_MODEL_PATH(model_path)
    , m_device(driver)
    , m_inference_result_ptr(new float) {
    auto available = m_core.get_available_devices();
    m_device =
        (std::find(available.begin(), available.end(), driver) != available.end()) ? driver : "CPU";
    RCLCPP_WARN_EXPRESSION(
        rclcpp::get_logger("armor_detector"), m_device != driver, "%s is not available, use CPU",
        driver.c_str());
    RCLCPP_INFO_STREAM(
        rclcpp::get_logger("armor_detecotr"),
        fmt::format("Inference available devices: {}", available));
    RCLCPP_INFO(rclcpp::get_logger("armor_detecotr"), "Inference device: %s", m_device.c_str());
    initModel(m_MODEL_PATH);
}

void Inference::initModel(const std::string& model_path) {
    m_core.set_property(ov::cache_dir(m_CACHE_DIR));
    m_model          = m_core.read_model(model_path);
    m_compiled_model = m_core.compile_model(m_model, m_device);
    m_infer_request  = m_compiled_model.create_infer_request();
    //    DLOG(INFO) << getInputAndOutputsInfo(*m_model);
    RCLCPP_INFO(
        rclcpp::get_logger("armor_detector"), "(%s)Model loading completed!", model_path.c_str());
}

bool Inference::inference(const cv::Mat& src, std::vector<InferenceResult>* inference_armors) {
    m_inference_result_ptr = nullptr;
    // 预处理 (en: pretreatment)
    cv::Mat resized_img = scaledResize(src, m_transformation_matrix);
    cv::Mat img_32f;
    cv::Mat split_img[3];
    resized_img.convertTo(img_32f, CV_32F);
    cv::split(img_32f, split_img);
    // 设置输入张量以运行推理(en: Set input tensor to run inference)
    ov::Tensor input_tensor = m_infer_request.get_input_tensor();
    int offset              = INPUT_WIDTH * INPUT_HEIGHT;
    for (int i = 0; i < 3; ++i)
        std::memcpy(
            input_tensor.data<float>() + offset * i, split_img[i].data,
            INPUT_WIDTH * INPUT_HEIGHT * sizeof(float));
    m_infer_request.set_input_tensor(input_tensor);
    m_infer_request.infer();
    // 解析推理结果(en: Analytic inference results)
    int src_width            = src.cols;
    int src_height           = src.cols;
    ov::Tensor output_tensor = m_infer_request.get_output_tensor();
    m_inference_result_ptr   = output_tensor.data<float>();
    decodeOutputs(
        m_inference_result_ptr, src_width, src_height, m_transformation_matrix, inference_armors);

    return !inference_armors->empty();
    ;
}
} // namespace armor_auto_aim