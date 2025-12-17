#include "ov_armor_tup.hpp"

// STD
#include <algorithm>
#include <cmath>
#include <cstring>

// 3rd
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

// OpenVINO
#include <openvino/openvino.hpp>

// ros2
#include <rclcpp/rclcpp.hpp>


namespace rm_auto_aim {

// =============== 预处理 / 后处理 ===============
std::unique_ptr<OVModelBase::PreprocContext> OVArmorTUP::preprocess(
    const cv::Mat& src, ov::InferRequest& request) {
    auto ctx = std::make_unique<Ctx>();

    // 1) letterbox & 记录还原矩阵（与 TUP 版一致）
    cv::Mat pr = scaledResize<INPUT_H, INPUT_W>(src, ctx->T);

    // 2) NHWC(BGR) -> NCHW float32
    // ★ 注意：不做 /255，模型训练就是吃 0~255 的 float
    cv::Mat f32;
    pr.convertTo(f32, CV_32F);
    cv::Mat ch[3];
    cv::split(f32, ch);

    // 3) 输入 Tensor（1x3xHxW）
    auto ip         = compiled_.input();
    const auto& shp = ip.get_shape(); // 期望 [1,3,416,416]
    if (shp.size() != 4 || shp[2] != INPUT_H || shp[3] != INPUT_W) {
        RCLCPP_ERROR(
            rclcpp::get_logger("armor_detector_ov"),
            "Input shape mismatch, get [%zu,%zu,%zu,%zu], expect [1,3,%d,%d]",
            shp.size() > 0 ? shp[0] : 0, shp.size() > 1 ? shp[1] : 0, shp.size() > 2 ? shp[2] : 0,
            shp.size() > 3 ? shp[3] : 0, INPUT_H, INPUT_W);
        return nullptr;
    }

    ov::Tensor in(ip.get_element_type(), shp);
    float* dst         = in.data<float>();
    const size_t plane = static_cast<size_t>(INPUT_W) * static_cast<size_t>(INPUT_H);

    std::memcpy(dst + plane * 0, ch[0].data, plane * sizeof(float));
    std::memcpy(dst + plane * 1, ch[1].data, plane * sizeof(float));
    std::memcpy(dst + plane * 2, ch[2].data, plane * sizeof(float));
    request.set_input_tensor(in);

    // 4) grids + 输出二维视图（anchors x columns）
    generate_grids_and_stride(INPUT_W, INPUT_H, {8, 16, 32}, ctx->grids);
    ctx->rows        = static_cast<int>(ctx->grids.size());
    ctx->cols        = 9 + NUM_COLORS + NUM_CLASSES; // [x1..y4(8), conf(1), colors(4), classes(8)]
    ctx->num_classes = NUM_CLASSES;

    const auto op     = compiled_.output();
    const auto& shape = op.get_shape();
    if (shape.size() == 3) {
        const int out_rows = static_cast<int>(shape[1]);
        const int out_cols = static_cast<int>(shape[2]);
        if (out_rows > 0) {
            if (out_rows != static_cast<int>(ctx->grids.size())) {
                RCLCPP_WARN(
                    rclcpp::get_logger("armor_detector_ov"),
                    "Output row size (%d) != generated grids (%zu), using min.", out_rows,
                    ctx->grids.size());
            }
            ctx->rows = std::min(out_rows, static_cast<int>(ctx->grids.size()));
        }
        if (out_cols > 0) {
            ctx->cols = out_cols;
        }
        const int derived_classes = ctx->cols - (9 + NUM_COLORS);
        if (derived_classes <= 0) {
            RCLCPP_ERROR(
                rclcpp::get_logger("armor_detector_ov"),
                "Unexpected output columns=%d (< base=%d).", ctx->cols, 9 + NUM_COLORS);
            return nullptr;
        }
        ctx->num_classes = derived_classes;
    } else {
        RCLCPP_WARN(
            rclcpp::get_logger("armor_detector_ov"),
            "Unexpected output rank=%zu, fallback to default cols=%d.", shape.size(), ctx->cols);
    }

    return ctx;
}

void OVArmorTUP::postprocess(
    const PreprocContext& ctx_, ov::InferRequest& request, std::vector<ArmorObject>& objects) {
    const auto& ctx = dynamic_cast<const Ctx&>(ctx_);

    ov::Tensor out = request.get_output_tensor(0);
    const float* p = out.data<const float>();
    objects.clear();
    if (!p)
        return;

    const size_t total  = out.get_size();
    const size_t expect = static_cast<size_t>(ctx.rows) * static_cast<size_t>(ctx.cols);
    if (total != expect) {
        RCLCPP_WARN(
            rclcpp::get_logger("armor_detector_ov"),
            "Skip frame: output size=%zu != expected=%zu (rows=%d, cols=%d).", total, expect,
            ctx.rows, ctx.cols);
        return;
    }

    cv::Mat obuf(ctx.rows, ctx.cols, CV_32F, const_cast<float*>(p));

    // —— 解码 —— //
    std::vector<ArmorObject> objs;
    objs.reserve(256);

    const int color_from = 9;
    const int color_to   = 9 + NUM_COLORS; // [9, 17)
    const int num_from   = color_to;       // 17
    const int num_to     = color_to + ctx.num_classes;
    if (num_to > ctx.cols) {
        RCLCPP_WARN(
            rclcpp::get_logger("armor_detector_ov"),
            "Skip frame: class columns (%d) exceed output width (%d).", num_to, ctx.cols);
        return;
    }

    auto finite = [](float x) { return std::isfinite(x); };

    static constexpr int RAW_CAP = 512;

    const int valid_rows = std::min(ctx.rows, static_cast<int>(ctx.grids.size()));
    for (int i = 0; i < valid_rows; ++i) {
        float conf = obuf.at<float>(i, 8);
        if (conf < conf_)
            continue;

        const auto& gs = ctx.grids[i];
        if (gs.stride <= 0 || gs.grid0 < 0 || gs.grid1 < 0)
            continue;

        // 顶点（网络尺度）-> 乘以 stride 并加网格
        float x1 =
            (obuf.at<float>(i, 0) + static_cast<float>(gs.grid0)) * static_cast<float>(gs.stride);
        float y1 =
            (obuf.at<float>(i, 1) + static_cast<float>(gs.grid1)) * static_cast<float>(gs.stride);
        float x2 =
            (obuf.at<float>(i, 2) + static_cast<float>(gs.grid0)) * static_cast<float>(gs.stride);
        float y2 =
            (obuf.at<float>(i, 3) + static_cast<float>(gs.grid1)) * static_cast<float>(gs.stride);
        float x3 =
            (obuf.at<float>(i, 4) + static_cast<float>(gs.grid0)) * static_cast<float>(gs.stride);
        float y3 =
            (obuf.at<float>(i, 5) + static_cast<float>(gs.grid1)) * static_cast<float>(gs.stride);
        float x4 =
            (obuf.at<float>(i, 6) + static_cast<float>(gs.grid0)) * static_cast<float>(gs.stride);
        float y4 =
            (obuf.at<float>(i, 7) + static_cast<float>(gs.grid1)) * static_cast<float>(gs.stride);

        if (!(finite(x1) && finite(y1) && finite(x2) && finite(y2) && finite(x3) && finite(y3)
              && finite(x4) && finite(y4)))
            continue;

        Eigen::Matrix<float, 3, 4> Pn;
        Pn << x1, x2, x3, x4, y1, y2, y3, y4, 1, 1, 1, 1;
        Eigen::Matrix<float, 3, 4> Pd = ctx.T * Pn;

        ArmorObject o{};
        for (int k = 0; k < 4; ++k) {
            o.apex[k] = cv::Point2f(Pd(0, k), Pd(1, k));
        }
        o.rect = cv::boundingRect(std::vector<cv::Point2f>(o.apex, o.apex + 4));
        if (!finite(o.rect.width) || !finite(o.rect.height))
            continue;

        const float polyA = area4(o.apex);
        if (!std::isfinite(polyA) || polyA <= 0.f)
            continue;

        // 颜色/数字分类
        cv::Mat color_scores = obuf.row(i).colRange(color_from, color_to);
        cv::Mat num_scores   = obuf.row(i).colRange(num_from, num_to);
        double color_score = 0, num_score = 0;
        cv::Point color_id{0, 0}, num_id{0, 0};
        cv::minMaxLoc(color_scores, nullptr, &color_score, nullptr, &color_id);
        cv::minMaxLoc(num_scores, nullptr, &num_score, nullptr, &num_id);

        o.cls   = num_id.x;
        o.color = color_id.x;
        o.prob  = conf;
        o.area  = static_cast<int>(polyA);

        objs.emplace_back(std::move(o));
        if ((int)objs.size() >= RAW_CAP)
            break; // 提前截断
    }

    // 正常流程：topK + NMS
    objects = topKAndNms(objs, topk_, nms_);
}

bool OVArmorTUP::detect(const cv::Mat& src, std::vector<ArmorObject>& objects) {
    if (!infer(src, objects)) {
        objects.clear();
        return false;
    }
    return !objects.empty();
}

// 注册到管理器
REGISTER_OV_MODEL("armor_tup", OVArmorTUP);

} // namespace rm_auto_aim
