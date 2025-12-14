#include "hik_camera_node.hpp"
#include "MvCameraControl.h"

#include <chrono>
#include <cinttypes>
#include <csignal>
#include <algorithm>
#include <cctype>

namespace hik_camera {

using namespace std::chrono_literals;

namespace {

bool parse_bayer_demosaic_method(const std::string& method, unsigned int& quality_out) {
    if (method.empty()) {
        return false;
    }

    std::string normalized = method;
    std::transform(
        normalized.begin(), normalized.end(), normalized.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (normalized == "0" || normalized == "nearest" || normalized == "nearest_neighbor" ||
        normalized == "nearest-neighbor" || normalized == "nn" || method == "最邻近" ||
        method == "最近邻") {
        quality_out = 0;
        return true;
    }

    if (normalized == "1" || normalized == "bilinear" || normalized == "linear" ||
        normalized == "bilinearity" || method == "双线性") {
        quality_out = 1;
        return true;
    }

    if (normalized == "2" || normalized == "optimized" || normalized == "optimal" ||
        normalized == "hamilton" || method == "最优化") {
        quality_out = 2;
        return true;
    }

    return false;
}

} // namespace

HikCameraNode::HikCameraNode(const rclcpp::NodeOptions& options)
    : Node("hik_camera", options) {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

    reconnect_interval_ms_  = this->declare_parameter("reconnect_interval_ms", 500);
    reconnect_max_attempts_ = this->declare_parameter("reconnect_max_attempts", -1);
    timestamp_offset_ms_    = this->declare_parameter("timestamp_offset_ms", 10);
    bayer_cvt_quality_      = this->declare_parameter("bayer_cvt_color_quality", 1);
    bayer_demosaic_method_  = this->declare_parameter<std::string>("bayer_demosaic_method", "");
    {
        unsigned int parsed = 0;
        if (!bayer_demosaic_method_.empty() &&
            parse_bayer_demosaic_method(bayer_demosaic_method_, parsed)) {
            bayer_cvt_quality_ = static_cast<int>(parsed);
        } else if (!bayer_demosaic_method_.empty()) {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid bayer_demosaic_method='%s' (expected: nearest|bilinear|optimized). "
                "Falling back to bayer_cvt_color_quality=%d.",
                bayer_demosaic_method_.c_str(), bayer_cvt_quality_);
        }

        if (bayer_cvt_quality_ < 0 || bayer_cvt_quality_ > 2) {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid bayer_cvt_color_quality=%d (expected 0/1/2). Falling back to 1 (bilinear).",
                bayer_cvt_quality_);
            bayer_cvt_quality_ = 1;
        }
    }
    force_8bit_pixel_format_ =
        this->declare_parameter("force_8bit_pixel_format", true); // 强制输出 8bit 像素格式
    // 尝试打开相机（第一次初始化）
    if (!openCamera()) {
        RCLCPP_WARN(this->get_logger(), "Initial camera open failed, entering reconnect loop.");
        // 如果第一次失败也可以选择阻塞等待重连，或直接继续并等待 capture 线程调用 reconnectLoop
    }

    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
    rmw_qos_profile_t image_profile =
        use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    auto image_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(image_profile), image_profile);
    image_pub_     = this->create_publisher<sensor_msgs::msg::Image>("image_raw", image_qos);
    camera_info_pub_ =
        this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", image_qos);

    declareParameters();

    // Load camera info (unchanged)
    camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
    camera_info_manager_ =
        std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url =
        this->declare_parameter("camera_info_url", "package://rm_auto_aim/config/camera_info.yaml");
    if (camera_info_manager_->validateURL(camera_info_url)) {
        camera_info_manager_->loadCameraInfo(camera_info_url);
        camera_info_template_ = camera_info_manager_->getCameraInfo();
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
        camera_info_template_ = sensor_msgs::msg::CameraInfo{};
    }
    prepareCameraInfoMessage();

    params_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

    // Capture thread
    capture_thread_ = std::thread{[this]() -> void {
        MV_FRAME_OUT out_frame;

        while (rclcpp::ok() && !exit_flag_.load()) {
            if (!image_msg_) {
                prepareImageMessage();
            }
            if (!camera_info_msg_) {
                prepareCameraInfoMessage();
            }

            {
                std::unique_lock lk(camera_mutex_);
                if (!camera_handle_) {
                    // 如果 handle 为空，先尝试重连（会阻塞直到成功或退出）
                    RCLCPP_WARN(this->get_logger(), "Camera handle is null, trying reconnect...");
                    lk.unlock();
                    if (!reconnectLoop()) {
                        // 如果 reconnectLoop 返回 false 说明程序要退出
                        break;
                    }
                    continue;
                }
                // 此处 camera_handle_ 非空，尝试抓帧
                image_msg_->header.stamp = this->now();

                nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
            } // unlock camera_mutex_ while processing buffer conversion to reduce hold time

            if (MV_OK == nRet) {
                // Convert & publish （注意 convert 时仍需保护 camera_handle_）
                {
                    std::scoped_lock lk(camera_mutex_);
                    convert_param_.pDstBuffer     = image_msg_->data.data();
                    convert_param_.nDstBufferSize = image_msg_->data.size();
                    convert_param_.pSrcData       = out_frame.pBufAddr;
                    convert_param_.nSrcDataLen    = out_frame.stFrameInfo.nFrameLen;
                    convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

                    MV_CC_ConvertPixelType(camera_handle_, &convert_param_);
                }

                image_msg_->height = out_frame.stFrameInfo.nHeight;
                image_msg_->width  = out_frame.stFrameInfo.nWidth;
                image_msg_->step   = out_frame.stFrameInfo.nWidth * 3;
                image_msg_->data.resize(image_msg_->width * image_msg_->height * 3);

                camera_info_msg_->header = image_msg_->header;
                // RCLCPP_INFO_SKIPFIRST_THROTTLE(
                //     this->get_logger(), *get_clock(), 1000,
                //     "Publishing image ptr=0x%" PRIXPTR " data_ptr=0x%" PRIXPTR
                //     " camera_info ptr=0x%" PRIXPTR,
                // reinterpret_cast<std::uintptr_t>(image_msg_.get()),
                // reinterpret_cast<std::uintptr_t>(image_msg_->data.data()),
                // reinterpret_cast<std::uintptr_t>(camera_info_msg_.get()));
                image_pub_->publish(std::move(image_msg_));
                camera_info_pub_->publish(*camera_info_msg_);

                {
                    std::scoped_lock lk(camera_mutex_);
                    MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
                }
                prepareImageMessage();
                fail_count = 0;
            } else {
                RCLCPP_WARN(this->get_logger(), "Get buffer failed! nRet: [%x]", nRet);
                RCLCPP_ERROR(
                    this->get_logger(), "Consecutive failures exceeded threshold, reconnecting...");
                if (!reconnectLoop()) {
                    break;
                }
            }

            // 退出检查
            if (exit_flag_.load())
                break;
        }
    }};
}

HikCameraNode::~HikCameraNode() {
    exit_flag_.store(true);
    if (capture_thread_.joinable())
        capture_thread_.join();
    {
        std::scoped_lock lk(camera_mutex_);
        if (camera_handle_) {
            MV_CC_StopGrabbing(camera_handle_);
            MV_CC_CloseDevice(camera_handle_);
            MV_CC_DestroyHandle(&camera_handle_);
            camera_handle_ = nullptr;
        }
    }
    RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
}

void HikCameraNode::declareParameters() {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    MVCC_FLOATVALUE f_value;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;
    // Exposure time
    param_desc.description = "Exposure time in microseconds";
    {
        std::scoped_lock lk(camera_mutex_);
        if (camera_handle_)
            MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
        else {
            // set some defaults if camera not ready
            f_value.fMin      = 1.0;
            f_value.fMax      = 1000000.0;
            f_value.fCurValue = 2500.0;
        }
    }
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value   = f_value.fMax;
    double exposure_time = this->declare_parameter("exposure_time", 2500, param_desc);
    {
        std::scoped_lock lk(camera_mutex_);
        if (camera_handle_)
            MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
    }
    RCLCPP_INFO(this->get_logger(), "Exposure time: %f", exposure_time);

    // Gain
    param_desc.description = "Gain";
    {
        std::scoped_lock lk(camera_mutex_);
        if (camera_handle_)
            MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
        else {
            f_value.fMin      = 0.0;
            f_value.fMax      = 48.0;
            f_value.fCurValue = 0.0;
        }
    }
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value   = f_value.fMax;
    double gain = this->declare_parameter("gain", f_value.fCurValue, param_desc);
    {
        std::scoped_lock lk(camera_mutex_);
        if (camera_handle_)
            MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
    }
    RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);
}

rcl_interfaces::msg::SetParametersResult
    HikCameraNode::parametersCallback(const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "";

    bool update_exposure = false;
    int exposure_time_us = 0;
    bool update_gain     = false;
    double gain_value    = 0.0;
    bool update_bayer    = false;
    unsigned int bayer_quality = 0;
    std::string bayer_method;

    for (const auto& param : parameters) {
        const auto& name = param.get_name();
        if (name == "exposure_time") {
            update_exposure = true;
            exposure_time_us = param.as_int();
        } else if (name == "gain") {
            update_gain = true;
            gain_value  = param.as_double();
        } else if (name == "bayer_cvt_color_quality") {
            const int q = param.as_int();
            if (q < 0 || q > 2) {
                result.successful = false;
                result.reason     = "bayer_cvt_color_quality must be 0(nearest)/1(bilinear)/2(optimized)";
                break;
            }
            update_bayer  = true;
            bayer_quality = static_cast<unsigned int>(q);
            bayer_method.clear();
        } else if (name == "bayer_demosaic_method") {
            bayer_method = param.as_string();
            unsigned int parsed = 0;
            if (!parse_bayer_demosaic_method(bayer_method, parsed)) {
                result.successful = false;
                result.reason     = "bayer_demosaic_method must be one of: nearest, bilinear, optimized";
                break;
            }
            update_bayer  = true;
            bayer_quality = parsed;
        } else {
            result.successful = false;
            result.reason     = "Unknown parameter: " + name;
            break;
        }
    }

    if (!result.successful) {
        return result;
    }

    std::scoped_lock lk(camera_mutex_);

    if (!camera_handle_ && (update_exposure || update_gain || update_bayer)) {
        result.successful = false;
        result.reason     = "Camera is not connected";
        return result;
    }

    if (update_exposure) {
        int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time_us);
        if (MV_OK != status) {
            result.successful = false;
            result.reason = "Failed to set exposure time, status = " + std::to_string(status);
            return result;
        }
    }

    if (update_gain) {
        int status = MV_CC_SetFloatValue(camera_handle_, "Gain", gain_value);
        if (MV_OK != status) {
            result.successful = false;
            result.reason     = "Failed to set gain, status = " + std::to_string(status);
            return result;
        }
    }

    if (update_bayer) {
        const int status =
            MV_CC_SetBayerCvtQuality(camera_handle_, static_cast<unsigned int>(bayer_quality));
        if (MV_OK != status) {
            result.successful = false;
            result.reason =
                "Failed to set Bayer demosaic method, status = " + std::to_string(status);
            return result;
        }
        bayer_cvt_quality_ = static_cast<int>(bayer_quality);
        bayer_demosaic_method_ = bayer_method;
    }
    return result;
}

void HikCameraNode::prepareImageMessage() {
    image_msg_                  = std::make_unique<sensor_msgs::msg::Image>();
    image_msg_->header.frame_id = "camera_optical_frame";
    image_msg_->encoding        = "rgb8";
    if (image_capacity_ > 0) {
        image_msg_->data.resize(image_capacity_);
    }
}

void HikCameraNode::prepareCameraInfoMessage() {
    camera_info_msg_ = std::make_unique<sensor_msgs::msg::CameraInfo>(camera_info_template_);
}

// ---------------------- camera helper impl ----------------------
bool HikCameraNode::openCamera() {
    std::scoped_lock lk(camera_mutex_);

    // 如果已有 handle，先关闭
    if (camera_handle_) {
        MV_CC_StopGrabbing(camera_handle_);
        MV_CC_CloseDevice(camera_handle_);
        MV_CC_DestroyHandle(&camera_handle_);
        camera_handle_ = nullptr;
    }

    MV_CC_DEVICE_INFO_LIST device_list;
    int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (ret != MV_OK || device_list.nDeviceNum == 0) {
        RCLCPP_WARN(this->get_logger(), "EnumDevices failed or no device found. ret=0x%x", ret);
        return false;
    }

    ret = MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
    if (ret != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "CreateHandle failed: 0x%x", ret);
        camera_handle_ = nullptr;
        return false;
    }

    ret = MV_CC_OpenDevice(camera_handle_);
    if (ret != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "OpenDevice failed: 0x%x", ret);
        MV_CC_DestroyHandle(&camera_handle_);
        camera_handle_ = nullptr;
        return false;
    }

    int bayer_ret = MV_CC_SetBayerCvtQuality(camera_handle_, static_cast<unsigned int>(bayer_cvt_quality_));
    if (bayer_ret != MV_OK) {
        RCLCPP_WARN(
            this->get_logger(), "SetBayerCvtQuality(%d) failed: 0x%x", bayer_cvt_quality_, bayer_ret);
    }

    if (force_8bit_pixel_format_) {
        MVCC_ENUMVALUE cur_pf{};
        int pf_ret = MV_CC_GetPixelFormat(camera_handle_, &cur_pf);
        if (pf_ret == MV_OK) {
            auto to_8bit = [](unsigned int current) -> unsigned int {
                switch (current) {
                case PixelType_Gvsp_BayerRG10:
                case PixelType_Gvsp_BayerRG10_Packed:
                case PixelType_Gvsp_BayerRG12:
                case PixelType_Gvsp_BayerRG12_Packed:
                case PixelType_Gvsp_BayerRG16: return PixelType_Gvsp_BayerRG8;
                case PixelType_Gvsp_BayerBG10:
                case PixelType_Gvsp_BayerBG10_Packed:
                case PixelType_Gvsp_BayerBG12:
                case PixelType_Gvsp_BayerBG12_Packed:
                case PixelType_Gvsp_BayerBG16: return PixelType_Gvsp_BayerBG8;
                case PixelType_Gvsp_BayerGB10:
                case PixelType_Gvsp_BayerGB10_Packed:
                case PixelType_Gvsp_BayerGB12:
                case PixelType_Gvsp_BayerGB12_Packed:
                case PixelType_Gvsp_BayerGB16: return PixelType_Gvsp_BayerGB8;
                case PixelType_Gvsp_BayerGR10:
                case PixelType_Gvsp_BayerGR10_Packed:
                case PixelType_Gvsp_BayerGR12:
                case PixelType_Gvsp_BayerGR12_Packed:
                case PixelType_Gvsp_BayerGR16: return PixelType_Gvsp_BayerGR8;
                case PixelType_Gvsp_Mono10:
                case PixelType_Gvsp_Mono10_Packed:
                case PixelType_Gvsp_Mono12:
                case PixelType_Gvsp_Mono12_Packed:
                case PixelType_Gvsp_Mono14:
                case PixelType_Gvsp_Mono16: return PixelType_Gvsp_Mono8;
                default: return current;
                }
            };
            unsigned int target_pf = to_8bit(cur_pf.nCurValue);
            if (target_pf != cur_pf.nCurValue) {
                int set_ret = MV_CC_SetPixelFormat(camera_handle_, target_pf);
                if (set_ret == MV_OK) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Force pixel format 0x%x -> 8bit 0x%x succeeded.",
                        cur_pf.nCurValue, target_pf);
                } else {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Force pixel format 0x%x -> 8bit 0x%x failed, ret=0x%x.",
                        cur_pf.nCurValue, target_pf, set_ret);
                }
            } else {
                RCLCPP_INFO(
                    this->get_logger(), "Pixel format already 8bit (0x%x).", cur_pf.nCurValue);
            }
        } else {
            RCLCPP_WARN(
                this->get_logger(), "GetPixelFormat failed, ret=0x%x, skip forcing 8bit.", pf_ret);
        }
    }

    // get image info & prepare buffers
    ret = MV_CC_GetImageInfo(camera_handle_, &img_info_);
    if (ret != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "GetImageInfo failed: 0x%x", ret);
        MV_CC_CloseDevice(camera_handle_);
        MV_CC_DestroyHandle(&camera_handle_);
        camera_handle_ = nullptr;
        return false;
    }

    image_capacity_ = static_cast<std::size_t>(img_info_.nHeightMax)
                    * static_cast<std::size_t>(img_info_.nWidthMax) * 3;
    prepareImageMessage();

    convert_param_.nWidth         = img_info_.nWidthValue;
    convert_param_.nHeight        = img_info_.nHeightValue;
    convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

    // start grabbing
    ret = MV_CC_StartGrabbing(camera_handle_);
    if (ret != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "StartGrabbing failed: 0x%x", ret);
        MV_CC_CloseDevice(camera_handle_);
        MV_CC_DestroyHandle(&camera_handle_);
        camera_handle_ = nullptr;
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Camera opened and started grabbing successfully.");
    return true;
}

void HikCameraNode::closeCamera() {
    std::scoped_lock lk(camera_mutex_);
    if (camera_handle_) {
        MV_CC_StopGrabbing(camera_handle_);
        MV_CC_CloseDevice(camera_handle_);
        MV_CC_DestroyHandle(&camera_handle_);
        camera_handle_ = nullptr;
        RCLCPP_INFO(this->get_logger(), "Camera closed.");
    }
}

bool HikCameraNode::reconnectLoop() {
    int attempts = 0;
    while (rclcpp::ok() && !exit_flag_.load()) {
        attempts++;
        RCLCPP_INFO(this->get_logger(), "Attempting to reconnect camera (attempt %d)...", attempts);
        if (openCamera()) {
            RCLCPP_INFO(this->get_logger(), "Reconnected camera after %d attempts.", attempts);
            return true;
        }
        if (reconnect_max_attempts_ > 0 && attempts >= reconnect_max_attempts_) {
            RCLCPP_ERROR(
                this->get_logger(), "Exceeded max reconnect attempts (%d).",
                reconnect_max_attempts_);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_interval_ms_));
    }
    return false;
}

} // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)
