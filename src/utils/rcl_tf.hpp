#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <optional>
#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace sleepy::utils {

// RclTF 统一封装 TF 查询/发布，并把 ROS TF 消息转换成 Eigen Isometry
// 参与几何计算。
class RclTF {
public:
  using Ptr = std::shared_ptr<RclTF>;

  RclTF(rclcpp::Node &n) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(n.get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(n);
    node_ = &n;
  }

  std::optional<tf2::Transform>
  get_tf2_transform(const std::string &target, const std::string &source,
                    rclcpp::Time t) const noexcept {
    try {
      // lookupTransform(target, source) 返回 source 坐标到 target 坐标的变换。
      auto tf_msg = tf_buffer_->lookupTransform(target, source, t);
      tf2::Transform tf;
      tf2::fromMsg(tf_msg.transform, tf);
      return tf;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(rclcpp::get_logger("tf"), "TF lookup failed: %s", ex.what());
      return std::nullopt;
    }
  }

  std::optional<tf2::Transform>
  get_tf2_transform(const std::string &target, const std::string &source,
                    rclcpp::Time t,
                    const rclcpp::Duration &timeout) const noexcept {
    try {
      auto tf_msg = tf_buffer_->lookupTransform(target, source, t, timeout);
      tf2::Transform tf;
      tf2::fromMsg(tf_msg.transform, tf);
      return tf;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(rclcpp::get_logger("tf"), "TF lookup failed: %s", ex.what());
      return std::nullopt;
    }
  }

  template <typename Scalar>
  using Isometry3 = Eigen::Transform<Scalar, 3, Eigen::Isometry>;

  template <typename Scalar>
  static Isometry3<Scalar>
  tf2eigen(const geometry_msgs::msg::TransformStamped &tf) noexcept {
    Isometry3<Scalar> T = Isometry3<Scalar>::Identity();

    // ROS 消息和平时 Eigen 计算之间统一通过 Isometry3 传递刚体变换。
    T.translation() << static_cast<Scalar>(tf.transform.translation.x),
        static_cast<Scalar>(tf.transform.translation.y),
        static_cast<Scalar>(tf.transform.translation.z);

    const auto &q = tf.transform.rotation;
    Eigen::Quaternion<Scalar> Q(
        static_cast<Scalar>(q.w), static_cast<Scalar>(q.x),
        static_cast<Scalar>(q.y), static_cast<Scalar>(q.z));

    Q.normalize();
    T.linear() = Q.toRotationMatrix();

    return T;
  }
  template <typename Scalar>
  static geometry_msgs::msg::Transform
  eigen2tf(const Isometry3<Scalar> &T) noexcept {
    geometry_msgs::msg::Transform msg;

    const auto &t = T.translation();
    msg.translation.x = static_cast<double>(t.x());
    msg.translation.y = static_cast<double>(t.y());
    msg.translation.z = static_cast<double>(t.z());

    Eigen::Quaternion<Scalar> q(T.rotation());
    q.normalize();

    msg.rotation.x = static_cast<double>(q.x());
    msg.rotation.y = static_cast<double>(q.y());
    msg.rotation.z = static_cast<double>(q.z());
    msg.rotation.w = static_cast<double>(q.w());

    return msg;
  }

  template <typename Scalar>
  std::optional<Isometry3<Scalar>>
  get_transform(const std::string &target, const std::string &source,
                rclcpp::Time t) const noexcept {
    try {
      auto tf_msg = tf_buffer_->lookupTransform(target, source, t);
      return tf2eigen<Scalar>(tf_msg);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(rclcpp::get_logger("tf"), "TF lookup failed: %s", ex.what());
      return std::nullopt;
    }
  }

  template <typename Scalar>
  std::optional<Isometry3<Scalar>>
  get_transform(const std::string &target, const std::string &source,
                rclcpp::Time t,
                const rclcpp::Duration &timeout) const noexcept {
    try {
      auto tf_msg = tf_buffer_->lookupTransform(target, source, t, timeout);
      return tf2eigen<Scalar>(tf_msg);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(rclcpp::get_logger("tf"), "TF lookup failed: %s", ex.what());
      return std::nullopt;
    }
  }

  template <typename Scalar>
  void publish_transform(const Isometry3<Scalar> &transform,
                         const std::string &parent_frame,
                         const std::string &child_frame,
                         const rclcpp::Time &stamp) const noexcept {
    // parent_frame -> child_frame 的语义保持和 TF 消息一致。
    geometry_msgs::msg::TransformStamped tmsg;
    tmsg.header.stamp = stamp;
    tmsg.header.frame_id = parent_frame;
    tmsg.child_frame_id = child_frame;

    const auto tr = transform.translation();
    Eigen::Quaternion<Scalar> q(transform.rotation());
    q.normalize();

    tmsg.transform.translation.x = static_cast<double>(tr.x());
    tmsg.transform.translation.y = static_cast<double>(tr.y());
    tmsg.transform.translation.z = static_cast<double>(tr.z());

    tmsg.transform.rotation.x = static_cast<double>(q.x());
    tmsg.transform.rotation.y = static_cast<double>(q.y());
    tmsg.transform.rotation.z = static_cast<double>(q.z());
    tmsg.transform.rotation.w = static_cast<double>(q.w());

    tf_broadcaster_->sendTransform(tmsg);
  }

  void publish_transform(
      const geometry_msgs::msg::TransformStamped &transform) const noexcept {
    tf_broadcaster_->sendTransform(transform);
  }

public:
  rclcpp::Node *node_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

} // namespace sleepy::utils
