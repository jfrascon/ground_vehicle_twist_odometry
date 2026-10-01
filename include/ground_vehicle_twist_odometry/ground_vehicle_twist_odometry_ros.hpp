#pragma once

#include <memory>
#include <string>
#include <type_traits>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "ground_vehicle_twist_odometry/ground_vehicle_twist_odometry.hpp"

namespace ground_vehicle_twist_odometry
{
  template<bool timestamped_twist>
  class GroundVehicleTwistOdometryROS: public rclcpp::Node
  {
      using TwistMessage = std::
        conditional_t<timestamped_twist, geometry_msgs::msg::TwistStamped, geometry_msgs::msg::Twist>;

    public:
      explicit GroundVehicleTwistOdometryROS(const std::string& node_name,
                                             const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
      void twist_callback(const typename TwistMessage::ConstSharedPtr msg);
      void reset_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>& request,
                          const std::shared_ptr<std_srvs::srv::Empty::Response>& response);
      void publish_odometry(const rclcpp::Time& stamp, const geometry_msgs::msg::Twist& twist);

      GroundVehicleTwistOdometry odometry_;
      typename rclcpp::Subscription<TwistMessage>::SharedPtr twist_sub_;
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
      tf2_ros::TransformBroadcaster tf_pub_;
      rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
      std::string odometry_frame_;
      std::string base_frame_;
  };

  extern template class GroundVehicleTwistOdometryROS<false>;
  extern template class GroundVehicleTwistOdometryROS<true>;
}  // namespace ground_vehicle_twist_odometry
