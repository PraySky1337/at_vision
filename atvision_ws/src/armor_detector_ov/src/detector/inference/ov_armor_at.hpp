#pragma once
#include "armor_detector_ov/ov_model_base.hpp"
#include "utils.hpp"

namespace rm_auto_aim {
struct OVArmorAT : public OVModelBase {
    static constexpr int INPUT_W     = 640;
    static constexpr int INPUT_H     = 640;
    static constexpr int NUM_COLORS  = 4;
    static constexpr int NUM_CLASSES = 13;
    static constexpr int NUM_KPTS    = 4; // yolo11pose: 4 corner keypoints (x,y,score)
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
        Eigen::Matrix<float, 3, 3> T;     // 输入->原图的逆映射（letterbox 还原）
        int input_w = INPUT_W;
        int input_h = INPUT_H;
        int nkpt    = NUM_KPTS;
    };

    std::unique_ptr<PreprocContext> preprocess(const cv::Mat& src) override;
    void postprocess(const PreprocContext& ctx_) override;

private:
    // 超参
    float conf_ = 0.75f;
    float nms_  = 0.30f;
    int topk_   = 128;

    // 输出缓存
    std::vector<ArmorObject> last_;
};
} // namespace rm_auto_aim
