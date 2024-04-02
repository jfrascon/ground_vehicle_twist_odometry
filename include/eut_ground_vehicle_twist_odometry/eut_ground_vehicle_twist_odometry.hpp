#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <type_traits>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

namespace eut_ground_vehicle_twist_odometry
{
  template<bool twist_is_timestamped>
  class GroundVehicleTwistOdometry: public rclcpp::Node
  {
    using twist_type = std::conditional_t<twist_is_timestamped,
                                          geometry_msgs::msg::TwistStamped,
                                          geometry_msgs::msg::Twist>;

    public:
    GroundVehicleTwistOdometry(const std::string& node_name,
                               const rclcpp::NodeOptions& options = rclcpp::NodeOptions()):
      rclcpp::Node{node_name, options},
      twist_sub_{this->create_subscription<twist_type>(
        this->declare_parameter("twist_topic", "twist"),
        10,
        std::bind(&GroundVehicleTwistOdometry::twist_cb, this, std::placeholders::_1))},
      odom_pub_{this->create_publisher<nav_msgs::msg::Odometry>(
        this->declare_parameter("odometry_topic", "odom"),
        10)},
      tf_pub_{*this},
      odometry_frame{this->declare_parameter("odometry_frame", "odom")},
      robot_base_frame{this->declare_parameter("robot_base_frame", "base_link")},
      publish_tf_{this->declare_parameter("publish_tf", true)}
    {
      // Wait some time for the first twist message to arrive.
      // The first twist message is the origin for the odometry computation.
      auto timeout_ms = static_cast<uint64_t>(
        std::round(1000.0 * std::abs(this->declare_parameter("first_twist_timeout", 5.0))));

      auto twist_msg = std::make_shared<twist_type>();

      if(!rclcpp::wait_for_message(*twist_msg,
                                   twist_sub_,
                                   this->get_node_options().context(),
                                   std::chrono::milliseconds{timeout_ms}))
      {
        throw std::runtime_error{"No first twist message received"};
      }

      if constexpr(twist_is_timestamped)
      {
        t_prev_ = twist_msg->header.stamp;
      }
      else
      {
        t_prev_ = this->now();
      }
    }

    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////

    void twist_cb(const typename twist_type::ConstSharedPtr twist_msg)
    {
      rclcpp::Time t_current;
      const geometry_msgs::msg::Twist* twist{nullptr};

      if constexpr(twist_is_timestamped)
      {
        t_current = twist_msg->header.stamp;
        twist     = &twist_msg->twist;
      }
      else
      {
        t_current = this->now();
        twist     = twist_msg.get();
      }

      // Why nanoseconds() instead of seconds(). According to the documentation in the file
      // /opt/ros/${ROS_DISTRO}/include/rclcpp/time.hpp:
      // * \warning Depending on sizeof(double) there could be significant precision loss.
      // * When an exact time is required use nanoseconds() instead.
      double dt = static_cast<double>((t_current - t_prev_).nanoseconds()) / 1E9;  // dt in seconds.

      // Check if we have received an old message that was dangling out there and is older than
      // the previous twist message that we received at time t_prev_ (t < t_prev_).
      // In this case no odometry step can be computed with the message we have just received.
      if(dt < 0.0)
      {
        RCLCPP_WARN(
          this->get_logger(),
          "Received twist msg from the past. No odometry step is computed with this message");

        return;
      }

      auto delta_x  = (twist->linear.x * std::cos(yaw_) - twist->linear.y * std::sin(yaw_)) * dt;
      auto delta_y  = (twist->linear.x * std::sin(yaw_) + twist->linear.y * std::cos(yaw_)) * dt;
      auto delta_th = twist->angular.z * dt;

      position_.x += delta_x;
      position_.y += delta_y;
      yaw_ += delta_th;

      double lin_cov = 1e-6;
      double ang_cov = 1e-4;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw_);
      q.normalize();

      nav_msgs::msg::Odometry odom;
      odom.header.stamp            = t_current;
      odom.header.frame_id         = odometry_frame;
      odom.child_frame_id          = robot_base_frame;
      odom.pose.pose.position      = position_;
      odom.pose.pose.orientation.x = q.x();
      odom.pose.pose.orientation.y = q.y();
      odom.pose.pose.orientation.z = q.z();
      odom.pose.pose.orientation.w = q.w();
      odom.twist.twist             = *twist;

      std::fill(std::begin(odom.pose.covariance), std::end(odom.pose.covariance), 0.0);

      odom.pose.covariance[0] = lin_cov;
      odom.pose.covariance[7] = lin_cov;
      odom.pose.covariance[14] = lin_cov;
      odom.pose.covariance[21] = ang_cov;
      odom.pose.covariance[28] = ang_cov;
      odom.pose.covariance[35] = ang_cov;

      std::fill(std::begin(odom.twist.covariance), std::end(odom.twist.covariance), 0.0);

      odom.twist.covariance[0]  = lin_cov / 2.0;
      odom.twist.covariance[7]  = lin_cov / 2.0;
      odom.twist.covariance[14] = lin_cov / 2.0;
      odom.twist.covariance[21] = ang_cov / 2.0;
      odom.twist.covariance[28] = ang_cov / 2.0;
      odom.twist.covariance[35] = ang_cov / 2.0;


      odom_pub_->publish(odom);

      if(publish_tf_)
      {
        geometry_msgs::msg::TransformStamped tfs;
        tfs.header.stamp            = t_current;
        tfs.header.frame_id         = odometry_frame;
        tfs.child_frame_id          = robot_base_frame;
        tfs.transform.translation.x = position_.x;
        tfs.transform.translation.y = position_.y;
        tfs.transform.translation.z = 0.0;
        tfs.transform.rotation      = odom.pose.pose.orientation;

        tf_pub_.sendTransform(tfs);
      }

      t_prev_ = t_current;
    }

    private:
    typename rclcpp::Subscription<twist_type>::SharedPtr twist_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    tf2_ros::TransformBroadcaster tf_pub_;
    std::string odometry_frame;
    std::string robot_base_frame;
    rclcpp::Time t_prev_;
    geometry_msgs::msg::Point position_;
    double yaw_;
    bool publish_tf_;
  };
}  // namespace eut_ground_vehicle_twist_odometry