#include "armor_detector/openvino_infer.hpp"
#include <openvino/runtime/properties.hpp>


static void nms_with_fusion(std::vector<Object>& objects,
                            std::vector<Object>& fused,
                            float nms_thresh = 0.5f,
                            float merge_iou  = 0.5f,
                            float merge_conf_err = 0.3f,
                            float center_dist_thr = 20.0f)
{
    fused.clear();
    if (objects.empty()) return;

    // 置信度排序
    std::sort(objects.begin(), objects.end(),
              [](const Object& a, const Object& b){ return a.prob > b.prob; });

    std::vector<bool> removed(objects.size(), false);

    for (size_t i = 0; i < objects.size(); i++) {
        if (removed[i]) continue;
        Object base = objects[i];

        for (size_t j = i+1; j < objects.size(); j++) {
            if (removed[j]) continue;

            float inter = (base.rect & objects[j].rect).area();
            float uni   = base.rect.area() + objects[j].rect.area() - inter + 1e-6f;
            float iou   = inter / uni;

            // 中心点距离
            cv::Point2f ci(base.rect.x + base.rect.width/2.f,
                           base.rect.y + base.rect.height/2.f);
            cv::Point2f cj(objects[j].rect.x + objects[j].rect.width/2.f,
                           objects[j].rect.y + objects[j].rect.height/2.f);
            float dist = cv::norm(ci - cj);

            if (iou > nms_thresh) {
                removed[j] = true;

                if ((iou > merge_iou || dist < center_dist_thr) &&
                    std::fabs(base.prob - objects[j].prob) < merge_conf_err &&
                    base.label == objects[j].label)
                {
                    // 把 j 的角点也加入 base
                    for (int k=0;k<4;k++) {
                        base.pts.emplace_back(objects[j].landmarks[2*k],
                                              objects[j].landmarks[2*k+1]);
                    }
                }
            }
        }

        // 融合角点
        if (base.pts.size() >= 8) {
            cv::Point2f avg[4] = {{0,0},{0,0},{0,0},{0,0}};
            int N = base.pts.size();
            for (int t=0;t<N;t++) {
                avg[t % 4] += base.pts[t];
            }
            for (int k=0;k<4;k++) {
                avg[k].x /= (N/4);
                avg[k].y /= (N/4);
                base.landmarks[2*k]   = avg[k].x;
                base.landmarks[2*k+1] = avg[k].y;
            }
            base.rect = cv::boundingRect(std::vector<cv::Point2f>(avg, avg+4));
        }

        fused.push_back(base);
    }
}



// Letterbox resize
cv::Mat OpenvinoInfer::letterbox(const cv::Mat& src, int& dw, int& dh, float& scale) {
    int w = src.cols, h = src.rows;
    scale     = std::min(input_w * 1.0f / w, input_h * 1.0f / h);
    int new_w = std::round(w * scale);
    int new_h = std::round(h * scale);
    dw        = (input_w - new_w) / 2;
    dh        = (input_h - new_h) / 2;

    cv::Mat resized;
    cv::resize(src, resized, cv::Size(new_w, new_h));
    cv::Mat out;
    cv::copyMakeBorder(
        resized, out, dh, input_h - new_h - dh, dw, input_w - new_w - dw, cv::BORDER_CONSTANT,
        {114, 114, 114});
    return out;
}

// IR 模型构造函数 (xml + bin)
OpenvinoInfer::OpenvinoInfer(
    std::string model_path_xml, std::string model_path_bin, std::string device) {
    model   = core.read_model(model_path_xml, model_path_bin);
    input_w = model->input().get_shape()[3]; // NCHW
    input_h = model->input().get_shape()[2];

    ppp = new ov::preprocess::PrePostProcessor(model);
    ppp->input()
        .tensor()
        .set_element_type(ov::element::u8)
        .set_layout("NHWC")
        .set_color_format(ov::preprocess::ColorFormat::BGR);
    ppp->input()
        .preprocess()
        .convert_element_type(ov::element::f32)
        .convert_color(ov::preprocess::ColorFormat::RGB)
        .scale({255., 255., 255.});
    ppp->input().model().set_layout("NCHW");
    ppp->output().tensor().set_element_type(ov::element::f32);
    model = ppp->build();

    compiled_model = core.compile_model(model, device, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
}

// ONNX 构造函数
OpenvinoInfer::OpenvinoInfer(std::string model_path, std::string device) {
    model   = core.read_model(model_path);
    input_w = model->input().get_shape()[3];
    input_h = model->input().get_shape()[2];

    ppp = new ov::preprocess::PrePostProcessor(model);
    ppp->input()
        .tensor()
        .set_element_type(ov::element::u8)
        .set_layout("NHWC")
        .set_color_format(ov::preprocess::ColorFormat::BGR);
    ppp->input()
        .preprocess()
        .resize(ov::preprocess::ResizeAlgorithm::RESIZE_LINEAR)
        .convert_element_type(ov::element::f32)
        .convert_color(ov::preprocess::ColorFormat::RGB)
        .scale({255., 255., 255.});
    ppp->input().model().set_layout("NCHW"); // 大多数 ONNX 模型是 NCHW
    ppp->output().tensor().set_element_type(ov::element::f32);
    model = ppp->build();

    compiled_model = core.compile_model(model, device);
}

// 推理接口
void OpenvinoInfer::infer(const cv::Mat& src, int detect_color) {
    objects.clear();
    tmp_objects.clear();
    ious.clear();

    // === 预处理：letterbox 并记录缩放参数 ===
    int dw, dh;
    float scale;
    cv::Mat img = letterbox(src, dw, dh, scale);

    // === 构建输入张量 ===
    ov::Tensor input_tensor(
        compiled_model.input().get_element_type(), compiled_model.input().get_shape(), img.data);

    ov::InferRequest infer_request = compiled_model.create_infer_request();
    infer_request.set_input_tensor(input_tensor);
    infer_request.infer();

    // === 后处理 ===
    auto output            = infer_request.get_output_tensor(0);
    ov::Shape output_shape = output.get_shape();
    cv::Mat output_buffer(output_shape[1], output_shape[2], CV_32F, output.data());

    constexpr float conf_threshold = 0.65;
    constexpr float nms_threshold  = 0.45;
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;

    for (int i = 0; i < output_buffer.rows; i++) {
        float confidence = sigmoid(output_buffer.at<float>(i, 8));
        if (confidence < conf_threshold)
            continue;

        cv::Mat color_scores   = output_buffer.row(i).colRange(9, 13);
        cv::Mat classes_scores = output_buffer.row(i).colRange(13, 22);
        cv::Point class_id, color_id;
        double score_color, score_num;
        cv::minMaxLoc(classes_scores, NULL, &score_num, NULL, &class_id);
        cv::minMaxLoc(color_scores, NULL, &score_color, NULL, &color_id);

        // if (color_id.x == 2 || color_id.x == 3)
        //     continue;
        // if (detect_color == 0 && color_id.x == 1)
        //     continue;
        // if (detect_color == 1 && color_id.x == 0)
        //     continue;

        Object obj;
        obj.prob  = confidence;
        obj.label = class_id.x;
        obj.color = color_id.x;

        // === landmarks 直接还原到原图坐标 ===
        std::vector<cv::Point2f> points;
        for (int k = 0; k < 4; k++) {
            float x = output_buffer.at<float>(i, 2 * k);
            float y = output_buffer.at<float>(i, 2 * k + 1);

            x = (x - dw) / scale;
            y = (y - dh) / scale;

            obj.landmarks[2 * k]     = x;
            obj.landmarks[2 * k + 1] = y;

            points.push_back(cv::Point2f(x, y));
        }

        cv::Rect rect = cv::boundingRect(points);
        obj.rect      = rect;

        objects.push_back(obj);
        boxes.push_back(rect);
        confidences.push_back(score_num);
    }
    nms_with_fusion(
        objects, tmp_objects, /*nms_thresh=*/nms_threshold,
        /*merge_iou=*/0.6f,
        /*merge_conf_err=*/0.3f);
}
