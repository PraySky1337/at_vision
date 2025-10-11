#ifndef OPENVINO_TEST_OPENVINOINFER_H
#define OPENVINO_TEST_OPENVINOINFER_H

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <vector>

struct Object {
    cv::Rect_<float> rect; 
    float landmarks[8];    // 四个关键点 (x0,y0, x1,y1, x2,y2, x3,y3)
    int label;
    float prob;
    int color;             
    double length;
    double width;
    double ratio;          

    std::vector<cv::Point2f> pts; //  存放多个候选角点
};

class OpenvinoInfer {
public:
    std::vector<Object> objects;
    std::vector<Object> tmp_objects;
    std::vector<double> ious;
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

    ov::Core core;
    std::shared_ptr<ov::Model> model;
    ov::preprocess::PrePostProcessor* ppp;
    ov::CompiledModel compiled_model;

    int input_w = 640;   // 模型输入宽
    int input_h = 640;   // 模型输入高

    OpenvinoInfer() {}

    // IR 模型构造函数 (xml + bin)
    OpenvinoInfer(std::string model_path_xml, std::string model_path_bin, std::string device);

    // ONNX 模型构造函数
    OpenvinoInfer(std::string model_path, std::string device);

    // 推理接口
    void infer(const cv::Mat& img, int detect_color);

    inline double sigmoid(double x) {
        if (x >= 0) return 1.0 / (1.0 + std::exp(-x));
        else return std::exp(x) / (1.0 + std::exp(x));
    }

    ~OpenvinoInfer() { delete ppp; }

private:
    cv::Mat letterbox(const cv::Mat& src, int& dw, int& dh, float& scale);
};

#endif // OPENVINO_TEST_OPENVINOINFER_H
