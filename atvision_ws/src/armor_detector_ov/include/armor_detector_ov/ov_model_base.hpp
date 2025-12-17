#pragma once

#include <Eigen/Core>
#include <functional>
#include <iostream>
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
        const std::string& model_path, const std::string& device = "AUTO",
        bool enable_profiling = false, bool enable_multi_thread = false,
        const std::string& device_priorities = "GPU,CPU") {
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

            ov::AnyMap compile_opts;
            compile_opts.emplace(
                ov::hint::performance_mode.name(),
                enable_multi_thread ? ov::hint::PerformanceMode::THROUGHPUT
                                    : ov::hint::PerformanceMode::LATENCY);
            if (enable_multi_thread) {
                compile_opts.emplace(ov::num_streams.name(), ov::streams::AUTO);
            } else {
                compile_opts.emplace(ov::num_streams.name(), 1);
            }
            if (!device_priorities.empty()
                && (device == "AUTO" || device.rfind("AUTO:", 0) == 0 || device == "MULTI"
                    || device.rfind("MULTI:", 0) == 0 || device == "HETERO"
                    || device.rfind("HETERO:", 0) == 0)) {
                compile_opts.emplace(ov::device::priorities.name(), device_priorities);
            }

            compiled_ = core_.compile_model(model_, device, compile_opts);
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
        std::vector<ArmorObject> tmp;
        if (!infer(src, tmp))
            return false;
        return true;
    }

    // 同步推理：输出检测结果
    bool infer(const cv::Mat& src, std::vector<ArmorObject>& objects) {
        objects.clear();
        if (src.empty())
            return false;
        auto ctx = preprocess(src, request_); // 子类负责：设置输入 Tensor
        if (!ctx)
            return false;
        request_.infer();
        postprocess(*ctx, request_, objects);
        return true;
    }

    // 创建额外的 InferRequest（用于异步/并行流水线）
    ov::InferRequest createInferRequest() { return compiled_.create_infer_request(); }

    // 查询实际执行设备（AUTO/MULTI 等会返回真实 device list）
    std::vector<std::string> executionDevices() const {
        try {
            return compiled_.get_property(ov::execution_devices);
        } catch (...) {
            return {};
        }
    }

public:
    // 子类必须实现
    virtual std::unique_ptr<PreprocContext> preprocess(const cv::Mat& src, ov::InferRequest& request) = 0;
    virtual void postprocess(
        const PreprocContext& ctx, ov::InferRequest& request, std::vector<ArmorObject>& objects) = 0;

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
