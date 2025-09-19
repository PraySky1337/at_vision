/**
 * @projectName armor_auto_aim
 * @file inference.cpp
 * @brief 推理
 * 
 * @author yx 
 * @date 2023-10-27 20:05
 */

#pragma once


#include <string>

#include <Eigen/Dense>
#include <openvino/openvino.hpp>

#include "armor_detector/parser.hpp"

namespace armor_auto_aim {
class Inference {
public:
    static constexpr int INPUT_WIDTH = 416;
    static constexpr int INPUT_HEIGHT = 416;

    Inference();

    explicit Inference(const std::string& model_path);

    Inference(const std::string& model_path, const std::string& driver);

    void initModel(const std::string& model_path);

    void setLabelUrl(const std::string& label_path);

    bool inference(const cv::Mat& src, std::vector<InferenceResult>* inference_armors);

    std::vector<std::string> labels_lookup;
private:
    const std::string m_MODEL_PATH = "../../model/opt-0527-001.xml";
    const std::string m_CACHE_DIR = "../.cache";

    std::string m_device = "CPU";
    ov::Core m_core;
    std::shared_ptr<ov::Model> m_model;
    ov::CompiledModel m_compiled_model;
    float* m_inference_result_ptr{};
    ov::InferRequest m_infer_request;
    Eigen::Matrix<float, 3, 3> m_transformation_matrix;
};

} // armor_auto_aim
