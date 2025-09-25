#include "hik_camera_node.hpp"

#include <chrono>
#include <csignal>

#include <image_transport/image_transport.hpp>

namespace hik_camera {

using namespace std::chrono_literals;

HikCameraNode::HikCameraNode(const rclcpp::NodeOptions& options)
    : Node("hik_camera", options) {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

    // 读取重连参数（可被动态参数覆盖）
    reconnect_interval_ms_  = this->declare_parameter("reconnect_interval_ms", 2000);
    reconnect_max_attempts_ = this->declare_parameter("reconnect_max_attempts", -1);

    // 尝试打开相机（第一次初始化）
    if (!openCamera()) {
        RCLCPP_WARN(this->get_logger(), "Initial camera open failed, entering reconnect loop.");
        // 如果第一次失败也可以选择阻塞等待重连，或直接继续并等待 capture 线程调用 reconnectLoop
    }

    // image_msg_ 预分配（如果在 openCamera 成功会在内部设置）
    image_msg_.header.frame_id = "camera_optical_frame";
    image_msg_.encoding        = "rgb8";

    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
    auto qos    = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

    declareParameters();

    // Load camera info (unchanged)
    camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
    camera_info_manager_ =
        std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url =
        this->declare_parameter("camera_info_url", "package://rm_auto_aim/config/camera_info.yaml");
    if (camera_info_manager_->validateURL(camera_info_url)) {
        camera_info_manager_->loadCameraInfo(camera_info_url);
        camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }

    params_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

    // Capture thread
    capture_thread_ = std::thread{[this]() -> void {
        MV_FRAME_OUT out_frame;

        RCLCPP_INFO(this->get_logger(), "Publishing image!");

        while (rclcpp::ok() && !exit_flag_.load()) {
            {
                std::scoped_lock lk(camera_mutex_);
                if (!camera_handle_) {
                    // 如果 handle 为空，先尝试重连（会阻塞直到成功或退出）
                    RCLCPP_WARN(this->get_logger(), "Camera handle is null, trying reconnect...");
                    if (!reconnectLoop()) {
                        // 如果 reconnectLoop 返回 false 说明程序要退出
                        break;
                    }
                }
                // 此处 camera_handle_ 非空，尝试抓帧
                nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
            } // unlock camera_mutex_ while processing buffer conversion to reduce hold time

            if (MV_OK == nRet) {
                // Convert & publish （注意 convert 时仍需保护 camera_handle_）
                {
                    std::scoped_lock lk(camera_mutex_);
                    convert_param_.pDstBuffer     = image_msg_.data.data();
                    convert_param_.nDstBufferSize = image_msg_.data.size();
                    convert_param_.pSrcData       = out_frame.pBufAddr;
                    convert_param_.nSrcDataLen    = out_frame.stFrameInfo.nFrameLen;
                    convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

                    MV_CC_ConvertPixelType(camera_handle_, &convert_param_);
                }

                image_msg_.header.stamp = this->now();
                image_msg_.height       = out_frame.stFrameInfo.nHeight;
                image_msg_.width        = out_frame.stFrameInfo.nWidth;
                image_msg_.step         = out_frame.stFrameInfo.nWidth * 3;
                image_msg_.data.resize(image_msg_.width * image_msg_.height * 3);

                camera_info_msg_.header = image_msg_.header;
                camera_pub_.publish(image_msg_, camera_info_msg_);

                {
                    std::scoped_lock lk(camera_mutex_);
                    MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
                }
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
    for (const auto& param : parameters) {
        if (param.get_name() == "exposure_time") {
            std::scoped_lock lk(camera_mutex_);
            int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
            if (MV_OK != status) {
                result.successful = false;
                result.reason = "Failed to set exposure time, status = " + std::to_string(status);
            }
        } else if (param.get_name() == "gain") {
            std::scoped_lock lk(camera_mutex_);
            int status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
            if (MV_OK != status) {
                result.successful = false;
                result.reason     = "Failed to set gain, status = " + std::to_string(status);
            }
        } else {
            result.successful = false;
            result.reason     = "Unknown parameter: " + param.get_name();
        }
    }
    return result;
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

    // get image info & prepare buffers
    ret = MV_CC_GetImageInfo(camera_handle_, &img_info_);
    if (ret != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "GetImageInfo failed: 0x%x", ret);
        MV_CC_CloseDevice(camera_handle_);
        MV_CC_DestroyHandle(&camera_handle_);
        camera_handle_ = nullptr;
        return false;
    }

    image_msg_.data.resize(img_info_.nHeightMax * img_info_.nWidthMax * 3);

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
