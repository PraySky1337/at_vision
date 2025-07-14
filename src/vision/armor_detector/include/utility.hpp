#pragma once
#include <condition_variable>
#include <geometry_msgs/msg/point.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
/**
 * @brief 初始化一个球形 Marker
 *
 * @param frame_id  坐标系 id，例如 "map"
 * @param id        同一 namespace 下的唯一 id
 * @param position  球心位置
 * @param diameter  直径（m）
 * @param color     颜色（RGBA，取值 0–1）
 * @param stamp     时间戳，通常传 this->now()
 * @param lifetime  生存时间，默认 0 表示永久
 * @param ns        namespace，用于 RViz 分组，可选
 * @return 完整配置好的 Marker
 */
inline visualization_msgs::msg::Marker initSphereMarker(
    const std::string& frame_id, int32_t id, const geometry_msgs::msg::Point& position,
    double diameter, const std_msgs::msg::ColorRGBA& color, const rclcpp::Time& stamp,
    const rclcpp::Duration& lifetime = rclcpp::Duration{0, 0}, const std::string& ns = "sphere") {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp    = stamp;

    marker.ns     = ns;
    marker.id     = id;
    marker.type   = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position      = position;
    marker.pose.orientation.w = 1.0; // 单位四元数

    marker.scale.x = marker.scale.y = marker.scale.z = diameter;

    marker.color = color;

    marker.lifetime = lifetime;      // 0 = 永久

    // 可选：使 RViz 在深度测试时总是可见
    // marker.frame_locked = false;
    // marker.mesh_use_embedded_materials = false;

    return marker;
}

template <typename T>
class LatestSlot {
public:
    // push 覆盖旧值（单生产者）
    void push(std::shared_ptr<T> p) { ptr_.store(p, std::memory_order_release); }

    // pop 取最新值（单消费者）
    std::shared_ptr<T> pop() { return ptr_.exchange(nullptr, std::memory_order_acq_rel); }

    bool empty() const { return ptr_.load(std::memory_order_acquire) == nullptr; }

private:
    std::atomic<std::shared_ptr<T>> ptr_{nullptr}; // 容量永远 = 1
};