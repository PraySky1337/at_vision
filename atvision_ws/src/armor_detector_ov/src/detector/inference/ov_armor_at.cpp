#include "ov_armor_at.hpp"

// STD
#include <algorithm>
#include <cmath>
#include <cstring>

// OpenCV
#include <opencv2/imgproc.hpp>

// ros2
#include <rclcpp/logging.hpp>

namespace rm_auto_aim {
std::unique_ptr<OVModelBase::PreprocContext> OVArmorAT::preprocess(const cv::Mat& src) {
    auto ctx = std::make_unique<Ctx>();

    // 1) 读输入 shape
    auto ip         = compiled_.input();
    const auto& shp = ip.get_shape(); // 期望 [1,3,H,W]
    if (shp.size() != 4 || shp[1] != 3) {
        RCLCPP_ERROR(
            rclcpp::get_logger("armor_detector_ov"),
            "Input rank mismatch, get [%zu,%zu,%zu,%zu], expect [1,3,H,W].",
            shp.size() > 0 ? shp[0] : 0, shp.size() > 1 ? shp[1] : 0, shp.size() > 2 ? shp[2] : 0,
            shp.size() > 3 ? shp[3] : 0);
        return nullptr;
    }
    ctx->input_h = static_cast<int>(shp[2]);
    ctx->input_w = static_cast<int>(shp[3]);

    // 2) letterbox（动态尺寸）+ 记录还原矩阵
    const float r = std::min(
        static_cast<float>(ctx->input_w) / static_cast<float>(src.cols),
        static_cast<float>(ctx->input_h) / static_cast<float>(src.rows));
    const int unpad_w = static_cast<int>(r * static_cast<float>(src.cols));
    const int unpad_h = static_cast<int>(r * static_cast<float>(src.rows));

    int dw = ctx->input_w - unpad_w;
    int dh = ctx->input_h - unpad_h;
    dw /= 2;
    dh /= 2;

    ctx->T << 1.f / r, 0.f, -static_cast<float>(dw) / r, 0.f, 1.f / r, -static_cast<float>(dh) / r,
        0.f, 0.f, 1.f;

    cv::Mat resized;
    cv::resize(src, resized, cv::Size(unpad_w, unpad_h));
    cv::Mat pr;
    cv::copyMakeBorder(resized, pr, dh, dh, dw, dw, cv::BORDER_CONSTANT);

    // 3) BGR -> RGB, /255, NHWC -> NCHW
    cv::Mat rgb;
    cv::cvtColor(pr, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(rgb, CV_32F, 1.f / 255.f);
    cv::Mat ch[3];
    cv::split(rgb, ch);

    ov::Tensor in(ip.get_element_type(), shp);
    float* dst         = in.data<float>();
    const size_t plane = static_cast<size_t>(ctx->input_w) * static_cast<size_t>(ctx->input_h);

    std::memcpy(dst + plane * 0, ch[0].data, plane * sizeof(float));
    std::memcpy(dst + plane * 1, ch[1].data, plane * sizeof(float));
    std::memcpy(dst + plane * 2, ch[2].data, plane * sizeof(float));
    request_.set_input_tensor(in);

    return ctx;
}

void OVArmorAT::postprocess(const PreprocContext& ctx_) {
    const auto& ctx = dynamic_cast<const Ctx&>(ctx_);

    ov::Tensor out = request_.get_output_tensor(0);
    const float* p = out.data<const float>();
    last_.clear();
    if (!p)
        return;

    const auto& shape = out.get_shape();

    auto map_point = [&](float x, float y) -> cv::Point2f {
        Eigen::Vector3f pt(x, y, 1.f);
        Eigen::Vector3f tr = ctx.T * pt;
        return {tr(0), tr(1)};
    };

    std::vector<ArmorObject> objs;
    objs.reserve(128);

    // 只认 YOLO11-pose OpenVINO 标准输出： [1, C, anchors] 或 [1, anchors, C]
    if (shape.size() != 3 || shape[0] != 1) {
        RCLCPP_WARN(
            rclcpp::get_logger("armor_detector_ov"),
            "Unexpected output rank/shape, expect [1,C,anchors] or [1,anchors,C], got rank=%zu",
            shape.size());
        return;
    }

    const int dim1    = static_cast<int>(shape[1]);
    const int dim2    = static_cast<int>(shape[2]);
    const int dims    = std::min(dim1, dim2); // per-anchor 维度
    const int anchors = std::max(dim1, dim2); // anchor 数

    const int nkpt    = ctx.nkpt;             // = 4
    const int kpt_dim = 2;                    // kpt_shape: [4, 2]
    const int nk      = nkpt * kpt_dim;       // = 8

    // dims = 4 + num_cls + nk
    const int num_cls = dims - 4 - nk;
    if (num_cls <= 0) {
        RCLCPP_ERROR(
            rclcpp::get_logger("armor_detector_ov"),
            "Invalid pose head dims=%d (nk=%d), num_cls<=0", dims, nk);
        return;
    }

    // 检查一下是否与配置一致（调试时很有用）
    const int total_cfg_cls = NUM_COLORS * NUM_CLASSES;
    if (total_cfg_cls != num_cls) {
        RCLCPP_WARN(
            rclcpp::get_logger("armor_detector_ov"),
            "Model nc (%d) != NUM_COLORS*NUM_CLASSES (%d), check yaml and constants.", num_cls,
            total_cfg_cls);
    }

    const bool channel_first = (dim1 < dim2); // true: [1, C, anchors]
    auto get                 = [&](int anchor, int d) -> float {
        if (channel_first) {
            return p[static_cast<size_t>(d) * anchors + anchor]; // [1, C, A]
        } else {
            return p[static_cast<size_t>(anchor) * dims + d];    // [1, A, C]
        }
    };

    for (int i = 0; i < anchors; ++i) {
        // 1) bbox: 已是解码后的 xywh，单位为输入图像像素
        const float cx = get(i, 0);
        const float cy = get(i, 1);
        const float w  = get(i, 2);
        const float h  = get(i, 3);

        if (!(std::isfinite(cx) && std::isfinite(cy) && std::isfinite(w) && std::isfinite(h)))
            continue;
        if (w <= 0.f || h <= 0.f)
            continue;

        // 2) 取最大类别概率作为 score
        float best_score = 0.f;
        int best_idx     = -1;
        for (int c = 0; c < num_cls; ++c) {
            float sc = get(i, 4 + c); // cls.sigmoid() 之后的概率
            if (sc > best_score) {
                best_score = sc;
                best_idx   = c;
            }
        }
        if (best_idx < 0 || best_score < conf_)
            continue;

        // 映射回 color + cls（你用 color * NUM_CLASSES + cls 映射）
        int combined = best_idx;
        int color    = combined / NUM_CLASSES; // 0..NUM_COLORS-1
        int cls      = combined % NUM_CLASSES; // 0..NUM_CLASSES-1

        // 3) cx,cy,w,h -> 四个角点（输入坐标系）
        const float x1 = cx - 0.5f * w;
        const float y1 = cy - 0.5f * h;
        const float x2 = cx + 0.5f * w;
        const float y2 = cy + 0.5f * h;

        ArmorObject o{};
        o.cls   = cls;
        o.color = color;
        o.prob  = best_score;

        o.apex[0] = map_point(x1, y1);
        o.apex[1] = map_point(x2, y1);
        o.apex[2] = map_point(x2, y2);
        o.apex[3] = map_point(x1, y2);

        // 4) keypoints：已经是像素坐标，无需再乘 input_w/input_h
        const int kp_base = 4 + num_cls;
        for (int k = 0; k < nkpt; ++k) {
            float kx = get(i, kp_base + 2 * k);
            float ky = get(i, kp_base + 2 * k + 1);
            if (!std::isfinite(kx) || !std::isfinite(ky))
                continue;
            o.apex[k] = map_point(kx, ky);
        }

        // 5) bbox 与面积
        o.rect = cv::boundingRect(std::vector<cv::Point2f>(o.apex, o.apex + 4));
        o.area = static_cast<int>(area4(o.apex));
        if (o.area > 0 && std::isfinite(o.rect.width) && std::isfinite(o.rect.height))
            objs.emplace_back(std::move(o));
    }

    // 外部 NMS
    last_ = topKAndNms(objs, topk_, nms_);
}

bool OVArmorAT::detect(const cv::Mat& src, std::vector<ArmorObject>& objects) {
    if (!run(src)) {
        objects.clear();
        return false;
    }
    objects = last_;
    return !objects.empty();
}

// 注册到管理器
REGISTER_OV_MODEL("armor_at", OVArmorAT);

} // namespace rm_auto_aim
