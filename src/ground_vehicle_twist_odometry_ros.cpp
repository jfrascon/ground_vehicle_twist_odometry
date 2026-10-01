#include "ground_vehicle_twist_odometry/ground_vehicle_twist_odometry_ros.hpp"

#include <cmath>
#include <cstddef>
#include <exception>
#include <memory>
#include <string>
#include <utility>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/time.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include "ground_vehicle_twist_odometry/ground_vehicle_twist_odometry.hpp"

namespace ground_vehicle_twist_odometry
{
  namespace
  {
    constexpr double kLinearCovariance{1e-6};
    constexpr double kAngularCovariance{1e-4};
    constexpr double kDefaultIncomingTwistRate{40.0};
    constexpr double kTwistCovarianceScale{0.5};
    constexpr std::size_t kQueueDepth{10};
    constexpr std::size_t kCovarianceDimensions{6};

    /**
     * @brief Check every ROS twist component before forwarding the message to odometry consumers.
     *
     * @param twist The incoming velocity sample.
     * @return True when both integration and the published twist can use this message.
     */
    bool is_finite(const geometry_msgs::msg::Twist& twist)
    {
      // The integrator uses three components, but Odometry publishes all six unchanged.
      return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) && std::isfinite(twist.linear.z) &&
             std::isfinite(twist.angular.x) && std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
    }
  }  // namespace

  /**
   * @brief Create the ROS interfaces that adapt one Twist message type to the shared integrator.
   *
   * @tparam timestamped_twist True for TwistStamped; false for Twist.
   * @param node_name Name used for the ROS node.
   * @param options ROS options, including parameter overrides and remappings.
   */
  template<bool timestamped_twist>
  // ROS interface construction expands into branches that are outside this constructor's logic.
  // NOLINTNEXTLINE
  GroundVehicleTwistOdometryROS<timestamped_twist>::GroundVehicleTwistOdometryROS(const std::string& node_name,
                                                                                  const rclcpp::NodeOptions& options):
    rclcpp::Node{node_name, options},
    tf_pub_{*this}
  {
    // ROS parameters and interfaces are available only after the base Node is constructed.
    // NOLINTBEGIN(cppcoreguidelines-prefer-member-initializer)
    declare_parameter("odometry_frame", "odom");
    declare_parameter("base_frame", "base_link");
    declare_parameter("publish_tf", true);
    declare_parameter("expected_incoming_twist_msg_rate", kDefaultIncomingTwistRate);

    // Frames are fixed for this node instance so odometry and TF use the same parent and child.
    odometry_frame_ = get_parameter("odometry_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();

    // Keep one subscription across initialization and reset; the integrator owns that state.
    twist_sub_ = create_subscription<TwistMessage>("twist",
                                                   kQueueDepth,
                                                   [this](typename TwistMessage::ConstSharedPtr msg) {
                                                     twist_callback(std::move(msg));
                                                   });
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", kQueueDepth);
    reset_srv_ = create_service<
      std_srvs::srv::Empty>("reset_odom",
                            [this](const std::shared_ptr<std_srvs::srv::Empty::Request>& request,
                                   const std::shared_ptr<std_srvs::srv::Empty::Response>& response) {
                              reset_callback(request, response);
                            });
    // NOLINTEND(cppcoreguidelines-prefer-member-initializer)

    RCLCPP_INFO(get_logger(), "GroundVehicleTwistOdometry node initialized");
  }

  /**
   * @brief Adapt one ROS velocity message and publish when it closes a valid time interval.
   *
   * @tparam timestamped_twist Selects the message timestamp or the node reception time.
   * @param msg Instantaneous velocity of the base origin in the base frame.
   */
  template<bool timestamped_twist>
  // Logging macros expand into branches that clang-tidy counts as callback decisions.
  // NOLINTNEXTLINE
  void GroundVehicleTwistOdometryROS<timestamped_twist>::twist_callback(const typename TwistMessage::ConstSharedPtr msg)
  {
    // TwistStamped carries measurement time; Twist can only use the node's reception time.
    rclcpp::Time stamp{};
    const geometry_msgs::msg::Twist* twist{nullptr};
    try
    {
      if constexpr(timestamped_twist)
      {
        stamp = rclcpp::Time{msg->header.stamp};
        twist = &msg->twist;
      }
      else
      {
        stamp = now();
        twist = msg.get();
      }
    }
    catch(const std::exception& error)
    {
      RCLCPP_WARN(get_logger(), "Cannot read twist timestamp: %s", error.what());
      return;
    }

    if(!is_finite(*twist))
    {
      // Reject the full ROS message before its unsupported components enter Odometry.
      RCLCPP_WARN(get_logger(), "Received twist with a non-finite velocity");
      return;
    }

    const VelocitySample sample{
      {twist->linear.x, twist->linear.y, twist->angular.z},
      stamp.nanoseconds(),
    };
    const UpdateResult result{odometry_.update(sample)};

    // The core reports whether a pose exists at this time; rejected samples produce no new TF.
    switch(result.status)
    {
      case UpdateStatus::Initialized:
        RCLCPP_INFO(get_logger(), "GroundVehicleTwistOdometry initialized at t = %.9f", stamp.seconds());
        return;
      case UpdateStatus::SameTimestamp:
        return;
      case UpdateStatus::InvalidVelocity:
        RCLCPP_WARN(get_logger(), "Received twist with a non-finite planar velocity");
        return;
      case UpdateStatus::BackwardsTime:
        RCLCPP_WARN(get_logger(), "Received twist msg from the past; no odometry step was computed");
        return;
      case UpdateStatus::NumericalError:
        RCLCPP_WARN(get_logger(), "Odometry integration produced a non-finite result");
        return;
      case UpdateStatus::Integrated:
        break;
    }

    // A slow producer raises an operational warning but does not shorten the integrated interval.
    const auto rate{1.0 / result.dt_seconds};
    const auto expected_rate{get_parameter("expected_incoming_twist_msg_rate").as_double()};
    if(rate < expected_rate)
    {
      RCLCPP_WARN(get_logger(),
                  "Received twist msg with a rate (%.2f Hz) lower than the expected incoming "
                  "twist msg rate (%.2f Hz).",
                  rate,
                  expected_rate);
    }

    publish_odometry(stamp, *twist);
  }

  /**
   * @brief Reset the pose and retained sample while keeping the ROS subscription active.
   *
   * @tparam timestamped_twist The message type used by this node instance.
   * @param request Empty request; reset needs no request data.
   * @param response Empty response; the service returns no state.
   */
  template<bool timestamped_twist>
  // ROS logging expands into branches beyond this callback's reset decision.
  // NOLINTNEXTLINE
  void GroundVehicleTwistOdometryROS<timestamped_twist>::reset_callback(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request>& request,
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Response>& response)
  {
    // The next valid message establishes the new time origin without publishing a zero pose.
    odometry_.reset();
    RCLCPP_INFO(get_logger(), "Odometry reset; waiting for the next twist message");
  }

  /**
   * @brief Publish the integrated pose and the newest instantaneous velocity at one sample time.
   *
   * @tparam timestamped_twist The message type used by this node instance.
   * @param stamp Time of the newest accepted velocity sample.
   * @param twist Full ROS velocity sample; integration used only its planar components.
   */
  template<bool timestamped_twist>
  void GroundVehicleTwistOdometryROS<timestamped_twist>::publish_odometry(const rclcpp::Time& stamp,
                                                                          const geometry_msgs::msg::Twist& twist)
  {
    // The core pose belongs to stamp, while the output twist is the sample measured at stamp.
    const Pose2D pose{odometry_.pose()};
    tf2::Quaternion orientation{};
    orientation.setRPY(0.0, 0.0, pose.yaw);
    orientation.normalize();

    nav_msgs::msg::Odometry odom{};
    odom.header.stamp = stamp;
    odom.header.frame_id = odometry_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.orientation.x = orientation.x();
    odom.pose.pose.orientation.y = orientation.y();
    odom.pose.pose.orientation.z = orientation.z();
    odom.pose.pose.orientation.w = orientation.w();
    odom.twist.twist = twist;

    // Preserve the existing fixed covariance values until source uncertainty is calibrated.
    for(std::size_t index{0}; index < kCovarianceDimensions; ++index)
    {
      const std::size_t diagonal{(kCovarianceDimensions * index) + index};
      const double covariance{(index < 3) ? kLinearCovariance : kAngularCovariance};
      odom.pose.covariance.at(diagonal) = covariance;
      odom.twist.covariance.at(diagonal) = covariance * kTwistCovarianceScale;
    }

    odom_pub_->publish(odom);

    if(get_parameter("publish_tf").as_bool())
    {
      // Reuse the odometry header and orientation so subscribers see the same pose in TF.
      geometry_msgs::msg::TransformStamped transform{};
      transform.header = odom.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = pose.x;
      transform.transform.translation.y = pose.y;
      transform.transform.rotation = odom.pose.pose.orientation;
      tf_pub_.sendTransform(transform);
    }
  }

  template class GroundVehicleTwistOdometryROS<false>;
  template class GroundVehicleTwistOdometryROS<true>;
}  // namespace ground_vehicle_twist_odometry
