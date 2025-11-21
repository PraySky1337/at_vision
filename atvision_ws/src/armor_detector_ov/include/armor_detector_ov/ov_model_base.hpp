#pragma once

#include <Eigen/Core>
#include <functional>
#include <memory>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <unordered_map>
#include <vector>
#include "armor_types.hpp"

namespace rm_auto_aim {


// ===== OpenVINO 抽象父类 =====
class OVModelBase {
public:
    struct PreprocContext {
        virtual ~PreprocContext() = default;
    };

    virtual ~OVModelBase() = default;

    // 加载模型
    virtual bool load(
        const std::string& model_path, const std::string& device = "CPU",
        bool enable_profiling = false) {
        try {
            if (enable_profiling)
                core_.set_property(device, ov::enable_profiling(true));
            model_ = core_.read_model(model_path);

            // 默认 NCHW float32（子类可在 preprocess 中自行组织 Tensor）
            ov::preprocess::PrePostProcessor ppp(model_);
            ppp.input().tensor().set_element_type(ov::element::f32).set_layout("NCHW");
            ppp.input().model().set_layout("NCHW");
            ppp.output().tensor().set_element_type(ov::element::f32);
            model_ = ppp.build();

            compiled_ = core_.compile_model(
                model_, device, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
            request_ = compiled_.create_infer_request();
            return true;
        } catch (const std::exception& e) {
            std::cerr << "[OVModelBase] load error: " << e.what() << std::endl;
            return false;
        }
    }

    // 一次完整推理：前处理→infer→后处理
    bool run(const cv::Mat& src) {
        if (src.empty())
            return false;
        auto ctx = preprocess(src); // 子类负责：设置输入 Tensor
        if (!ctx)
            return false;
        request_.infer();
        postprocess(*ctx);
        return true;
    }

protected:
    // 子类必须实现
    virtual std::unique_ptr<PreprocContext> preprocess(const cv::Mat& src) = 0;
    virtual void postprocess(const PreprocContext& ctx)                    = 0;

protected:
    ov::Core core_;
    std::shared_ptr<ov::Model> model_;
    ov::CompiledModel compiled_;
    ov::InferRequest request_;
};

class OVModelManager {
public:
    using Creator = std::function<std::unique_ptr<OVModelBase>()>;

    static OVModelManager& instance() {
        static OVModelManager inst;
        return inst;
    }

    void registerModel(const std::string& name, Creator c) { registry_[name] = std::move(c); }

    std::unique_ptr<OVModelBase> create(const std::string& name) const {
        auto it = registry_.find(name);
        if (it == registry_.end())
            return nullptr;
        return (it->second)();
    }

private:
    std::unordered_map<std::string, Creator> registry_;
};

// 注册宏（子类 .cpp 用）
#define REGISTER_OV_MODEL(NAME, TYPE)                          \
    static bool _reg_##TYPE = []() {                           \
        rm_auto_aim::OVModelManager::instance().registerModel( \
            NAME, []() { return std::make_unique<TYPE>(); });  \
        return true;                                           \
    }()

} // namespace rm_auto_aim
