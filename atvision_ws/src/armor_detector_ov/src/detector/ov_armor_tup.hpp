#pragma once
#include "armor_detector_ov/ov_model_base.hpp"

#include <Eigen/Core>
#include <memory>
#include <opencv2/core.hpp>
#include <vector>

namespace rm_auto_aim {

// baseline : yolov3 -> yolox
// TUP 装甲检测（实现 preprocess/postprocess）
class OVArmorTUP final : public OVModelBase {
public:
    struct GridAndStride {
        int grid0{}, grid1{}, stride{};
    };

    // ArmorObject
    bool detect(const cv::Mat& src, std::vector<ArmorObject>& objects);

    // threshold
    void setThreshold(float conf, float nms) {
        conf_ = conf;
        nms_  = nms;
    }
    void setTopK(int k) { topk_ = k; }

protected:
    struct Ctx : PreprocContext {
        Eigen::Matrix<float, 3, 3> T;      // 输入->原图的逆映射（letterbox 还原）
        std::vector<GridAndStride> grids;  // (g0,g1,stride)
        int rows = 0, cols = 0;            // 输出二维视图大小（anchors x columns）
        int num_classes = NUM_CLASSES;     // 实际可解析的类别列数
    };

    std::unique_ptr<PreprocContext> preprocess(const cv::Mat& src) override;
    void postprocess(const PreprocContext& ctx_) override;

private:
    // 工具
    static cv::Mat scaledResize(const cv::Mat& img, Eigen::Matrix<float, 3, 3>& T);
    static void generate_grids_and_stride(
        int W, int H, const std::vector<int>& strides, std::vector<GridAndStride>& gs);
    static float iou(const cv::Rect2f& a, const cv::Rect2f& b);
    static float area4(const cv::Point2f p[4]);
    static std::vector<ArmorObject> topKAndNms(std::vector<ArmorObject>& v, int topk, float nms);

private:
    // 模型头部配置（与 TUP 模型保持一致）
    static constexpr int INPUT_W     = 416;
    static constexpr int INPUT_H     = 416;
    static constexpr int NUM_CLASSES = 8;
    static constexpr int NUM_COLORS  = 4; 

    // 超参
    float conf_ = 0.75f;
    float nms_  = 0.30f;   // ★ NMS 与原版一致
    int topk_   = 128;

    // 输出缓存
    std::vector<ArmorObject> last_;
};

} // namespace rm_auto_aim
