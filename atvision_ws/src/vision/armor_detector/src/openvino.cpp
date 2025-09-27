#include "armor_detector/openvino.hpp"
#include <algorithm>
#include <cmath>
#include <opencv2/dnn.hpp>

namespace rm_auto_aim {

OpenVinoInference::OpenVinoInference(
    const std::string& xml, const std::string& device_name, float score_threshold, float nms_iou,
    float roi_expand, float roi_offset)
    : core()
    , model(core.read_model(xml))
    , compiled_model(core.compile_model(model, device_name))
    , infer_request(compiled_model.create_infer_request())
    , score_threshold_(score_threshold)
    , nms_iou_(nms_iou)
    , roi_expand_(roi_expand)
    , roi_offset_(roi_offset) {
    input  = compiled_model.input(0);
    output = compiled_model.output(0);
}

void OpenVinoInference::sortPoints(std::array<cv::Point2f, 4>& pts) {
    std::array<cv::Point2f, 4> p = pts;
    std::sort(
        p.begin(), p.end(), [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
    cv::Point2f top[2] = {p[0], p[1]};
    cv::Point2f bot[2] = {p[2], p[3]};
    if (top[0].x > top[1].x)
        std::swap(top[0], top[1]);
    if (bot[0].x > bot[1].x)
        std::swap(bot[0], bot[1]);
    pts[0] = top[0]; // TL
    pts[1] = bot[0]; // BL
    pts[2] = bot[1]; // BR
    pts[3] = top[1]; // TR
}

void OpenVinoInference::letterboxTopleft(
    const cv::Mat& src, int net_w, int net_h, cv::Mat& canvas, float& scale) {
    const int src_w = src.cols, src_h = src.rows;
    const float r = std::min(net_w / static_cast<float>(src_w), net_h / static_cast<float>(src_h));
    const int new_w = static_cast<int>(std::round(src_w * r));
    const int new_h = static_cast<int>(std::round(src_h * r));
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
    canvas = cv::Mat::zeros(net_h, net_w, src.type());
    resized.copyTo(canvas(cv::Rect(0, 0, new_w, new_h)));
    scale = r;       // 注意：这里返回的是 r，本函数与上文一致
}

OpenVinoInference::Result OpenVinoInference::infer(const cv::Mat& frame) {
    Result result;

    // 1) 预处理：左上对齐 letterbox + 1/255 + BGR->RGB
    const auto in_shape = input.get_shape(); // NCHW
    const int net_h     = static_cast<int>(in_shape[2]);
    const int net_w     = static_cast<int>(in_shape[3]);

    cv::Mat lb;
    float r = 1.f;
    letterboxTopleft(frame, net_w, net_h, lb, r);
    const float inv = (r > 0.f) ? (1.f / r) : 1.f;

    cv::Mat blob = cv::dnn::blobFromImage(
        lb, 1.0 / 255.0, cv::Size(net_w, net_h), cv::Scalar(), /*swapRB=*/true, /*crop=*/false);

    ov::Tensor in_tensor(input.get_element_type(), input.get_shape(), blob.ptr<float>());
    infer_request.set_input_tensor(in_tensor);
    infer_request.infer();

    // 2) 取输出并转成 (N,14)
    ov::Tensor out_tensor = infer_request.get_output_tensor(0);
    const auto& os        = out_tensor.get_shape(); // 期望 [1,14,N]
    const int C           = static_cast<int>(os[1]);
    const int N           = static_cast<int>(os[2]);
    const float* ptr      = out_tensor.data<const float>();

    cv::Mat m(C, N, CV_32F, const_cast<float*>(ptr));
    cv::Mat pred;
    cv::transpose(m, pred);                         // (N,14)

    // 3) 逐类收集候选
    struct Bucket {
        std::vector<cv::Rect> boxes;
        std::vector<float> scores;
        std::vector<std::array<cv::Point2f, 4>> kps;
    };
    Bucket buckets[2];

    const int W = frame.cols, H = frame.rows;

    for (int i = 0; i < pred.rows; ++i) {
        const float* rptr = pred.ptr<float>(i);

        // 类别分数（第4/5列）
        float s0 = rptr[4], s1 = rptr[5];
        int cid  = (s1 > s0) ? 1 : 0;
        float sc = (cid == 1) ? s1 : s0;
        if (sc < score_threshold_)
            continue;

        // 框：网络坐标 → 先做外扩/偏移 → 映射回原图（乘 inv）
        float cx = rptr[0], cy = rptr[1], w = rptr[2], h = rptr[3];
        float x  = (cx - 0.5f * w - roi_offset_) * inv;
        float y  = (cy - 0.5f * h - roi_offset_) * inv;
        float ww = (w * roi_expand_) * inv;
        float hh = (h * roi_expand_) * inv;

        int xi = std::clamp(static_cast<int>(std::lround(x)), 0, W - 1);
        int yi = std::clamp(static_cast<int>(std::lround(y)), 0, H - 1);
        int wi = std::clamp(static_cast<int>(std::lround(ww)), 1, W);
        int hi = std::clamp(static_cast<int>(std::lround(hh)), 1, H);
        // 防越界
        wi = std::min(wi, W - xi);
        hi = std::min(hi, H - yi);

        // 关键点：网络坐标 → 原图（乘 inv）
        std::array<cv::Point2f, 4> kp{};
        for (int k = 0; k < 4; ++k) {
            float px = rptr[6 + k * 2 + 0] * inv;
            float py = rptr[6 + k * 2 + 1] * inv;
            px       = std::clamp(px, 0.f, static_cast<float>(W - 1));
            py       = std::clamp(py, 0.f, static_cast<float>(H - 1));
            kp[k]    = cv::Point2f(px, py);
        }
        sortPoints(kp);

        buckets[cid].boxes.emplace_back(xi, yi, wi, hi);
        buckets[cid].scores.emplace_back(sc);
        buckets[cid].kps.emplace_back(kp);
    }

    // 4) 按类 NMS → 填充结果
    for (int cid = 0; cid < 2; ++cid) {
        const auto& B = buckets[cid].boxes;
        const auto& S = buckets[cid].scores;
        const auto& K = buckets[cid].kps;
        if (B.empty())
            continue;

        std::vector<int> keep;
        cv::dnn::NMSBoxes(B, S, score_threshold_, nms_iou_, keep);
        for (int idx : keep) {
            ArmorDet d;
            d.class_id = cid;
            d.score    = S[idx];
            d.box      = B[idx];
            for (int k = 0; k < 4; ++k)
                d.kps[k] = K[idx][k];
            result.detections.emplace_back(std::move(d));
        }
    }

    return result;
}

} // namespace rm_auto_aim
