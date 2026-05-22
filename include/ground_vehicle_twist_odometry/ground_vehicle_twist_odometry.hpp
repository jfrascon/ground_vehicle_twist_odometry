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
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace ground_vehicle_twist_odometry
{
  template<bool timestamped_twist>
  class GroundVehicleTwistOdometry: public rclcpp::Node
  {
    using twist_type = std::
      conditional_t<timestamped_twist, geometry_msgs::msg::TwistStamped, geometry_msgs::msg::Twist>;

    public:
    GroundVehicleTwistOdometry(const std::string& node_name,
                               const rclcpp::NodeOptions& options = rclcpp::NodeOptions()):
      rclcpp::Node{node_name, options},
      twist_sub_{
        create_subscription<twist_type>("twist",
                                        10,
                                        std::bind(&GroundVehicleTwistOdometry::init_cb, this, std::placeholders::_1))},
      odom_pub_{create_publisher<nav_msgs::msg::Odometry>("odom", 10)},
      logger_ctor_{get_logger().get_child("constructor")},
      logger_init_cb_{get_logger().get_child("init_cb")},
      logger_reset_cb_{get_logger().get_child("reset_cb")},
      logger_twist_cb_{get_logger().get_child("twist_cb")},
      tf_pub_{*this},
      reset_srv_{create_service<std_srvs::srv::Empty>(
        "reset_odom",
        std::bind(&GroundVehicleTwistOdometry::reset_cb, this, std::placeholders::_1, std::placeholders::_2))}
    {
      // Declare parameters.
      declare_parameter("odometry_frame", "odom");
      declare_parameter("base_frame", "base_link");
      declare_parameter("publish_tf", true);
      declare_parameter("expected_incoming_twist_msg_rate", 40.0);

      // Changes in parameters 'odometry_frame' and 'base_frame' during runtime are not taken into
      // account. This decision has been made because it does not make much sense to change these
      // parameters during runtime.
      odometry_frame_ = get_parameter("odometry_frame").as_string();
      base_frame_     = get_parameter("base_frame").as_string();

      RCLCPP_INFO(logger_ctor_, "GroundVehicleTwistOdometry node initialized");
    }

    //////////////////////////////////////////////////////////////////////////////

    /**
     * Initialization callback called upon reception of the first twist message.
     * The first twist message marks the initialization of the odometry computation (start of
     * integration) (Also after a reset service call). The moment the first twist message is
     * received, the member variable t_prev_ is initialized either with the timestamp contained in
     * the message (if timestamped_twist is true) or with the current node time (if
     * timestamped_twist is false). The t_prev_ variable is then used to compute the time difference
     * dt in the regular twist callback. Then, the twist subscription is replaced with a new one
     * that calls the regular twist callback.
     * @param msg The received twist message.
     *
     */
    void init_cb(const typename twist_type::ConstSharedPtr msg)
    {
      // Register first time stamp.
      // Depending on the type of twist message the node is instantiated with, either use the
      // timestamp contained in the message or the current node time.
      if constexpr(timestamped_twist)
      {
        t_prev_ = msg->header.stamp;
      }
      else
      {
        t_prev_ = this->now();
      }

      RCLCPP_INFO(logger_init_cb_, "GroundVehicleTwistOdometry initialized at t = %.9f", t_prev_.seconds());

      // Replace subscription with the regular twist callback, to receive twist messages and start
      // the integration process.
      twist_sub_.reset();
      twist_sub_ = this->create_subscription<twist_type>(
        "twist",
        10,
        std::bind(&GroundVehicleTwistOdometry::twist_cb, this, std::placeholders::_1));
    }

    //////////////////////////////////////////////////////////////////////////////

    void reset_cb(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
      RCLCPP_INFO(logger_reset_cb_, "Reset odom requested");

      // Reset position and orientation.
      this->reset();

      // Wait for the next twist message to re-initialize t_prev_, which marks the new odometry
      // computation. (Start of integration)
      twist_sub_.reset();
      twist_sub_ = this->create_subscription<twist_type>(
        "twist",
        10,
        std::bind(&GroundVehicleTwistOdometry::init_cb, this, std::placeholders::_1));

      RCLCPP_INFO(logger_reset_cb_, "Odometry reset; waiting for the next twist message");
    }

    void reset()
    {
      // Reset position and orientation.
      position_.x = 0.0;
      position_.y = 0.0;
      yaw_        = 0.0;
    }

    //////////////////////////////////////////////////////////////////////////////

    void twist_cb(const typename twist_type::ConstSharedPtr msg)
    {
      rclcpp::Time t_msg;
      const geometry_msgs::msg::Twist* twist{nullptr};

      if constexpr(timestamped_twist)
      {
        t_msg = msg->header.stamp;
        twist = &msg->twist;
      }
      else
      {
        t_msg = this->now();
        twist = msg.get();
      }

      // Why nanoseconds() instead of seconds(). According to the documentation in the file
      // /opt/ros/${ROS_DISTRO}/include/rclcpp/time.hpp:
      // * \warning Depending on sizeof(double) there could be significant precision loss.
      // * When an exact time is required use nanoseconds() instead.
      const auto dt{static_cast<double>((t_msg - t_prev_).nanoseconds()) / 1E9};  // dt in seconds.

      // RCLCPP_INFO(logger_twist_cb_,
      //             "[%.9f s] dt: %.9f s, v_x: %.3f m/s, v_y: %.3f m/s, w_z: %.3f "
      //             "rad/s",
      //             t_msg.seconds(),
      //             dt,
      //             twist->linear.x,
      //             twist->linear.y,
      //             twist->angular.z);

      // Check if we have received an old message that was dangling out there and is older than
      // the previous twist message that we received at time t_prev_ (t < t_prev_).
      // In this case no odometry step can be computed with the message we have just received.
      if(dt < 0.0)
      {
        RCLCPP_WARN(logger_twist_cb_,
                    "Received twist msg from the past. No odometry step is computed with this "
                    "message");

        RCLCPP_WARN(logger_twist_cb_, "t_msg: %.9f, t_prev_: %.9f", t_msg.seconds(), t_prev_.seconds());

        return;
      }

      const auto rate{1.0 / dt};
      // Either integer or double parameter type is accepted.
      const auto expected_incoming_twist_msg_rate{
        get_parameter("expected_incoming_twist_msg_rate").get_value<double>()};

      if(rate < expected_incoming_twist_msg_rate)
      {
        RCLCPP_WARN(logger_twist_cb_,
                    "Received twist msg with a rate (%.2f Hz) lower than the expected incoming "
                    "twist msg rate (%.2f Hz).",
                    rate,
                    expected_incoming_twist_msg_rate);
      }

      // Compute once, then reuse.
      const auto cos_yaw{std::cos(yaw_)};
      const auto sin_yaw{std::sin(yaw_)};

      const auto delta_x{(twist->linear.x * cos_yaw - twist->linear.y * sin_yaw) * dt};
      const auto delta_y{(twist->linear.x * sin_yaw + twist->linear.y * cos_yaw) * dt};
      const auto delta_th{twist->angular.z * dt};

      position_.x += delta_x;
      position_.y += delta_y;
      yaw_ += delta_th;

      // RCLCPP_INFO(logger_twist_cb_,
      //             "Odometry update x: %.6f m, y: %.6f m, th: %.6f rad",
      //             position_.x,
      //             position_.y,
      //             yaw_);

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw_);
      q.normalize();

      nav_msgs::msg::Odometry odom;
      odom.header.stamp            = t_msg;
      odom.header.frame_id         = odometry_frame_;
      odom.child_frame_id          = base_frame_;
      odom.pose.pose.position      = position_;
      odom.pose.pose.orientation.x = q.x();
      odom.pose.pose.orientation.y = q.y();
      odom.pose.pose.orientation.z = q.z();
      odom.pose.pose.orientation.w = q.w();
      odom.twist.twist             = *twist;

      std::fill(std::begin(odom.pose.covariance), std::end(odom.pose.covariance), 0.0);

      odom.pose.covariance[0]  = LIN_COV;
      odom.pose.covariance[7]  = LIN_COV;
      odom.pose.covariance[14] = LIN_COV;
      odom.pose.covariance[21] = ANG_COV;
      odom.pose.covariance[28] = ANG_COV;
      odom.pose.covariance[35] = ANG_COV;

      std::fill(std::begin(odom.twist.covariance), std::end(odom.twist.covariance), 0.0);

      odom.twist.covariance[0]  = LIN_COV / 2.0;
      odom.twist.covariance[7]  = LIN_COV / 2.0;
      odom.twist.covariance[14] = LIN_COV / 2.0;
      odom.twist.covariance[21] = ANG_COV / 2.0;
      odom.twist.covariance[28] = ANG_COV / 2.0;
      odom.twist.covariance[35] = ANG_COV / 2.0;

      odom_pub_->publish(odom);

      if(get_parameter("publish_tf").as_bool())
      {
        geometry_msgs::msg::TransformStamped tfs;
        tfs.header.stamp            = t_msg;
        tfs.header.frame_id         = odometry_frame_;
        tfs.child_frame_id          = base_frame_;
        tfs.transform.translation.x = position_.x;
        tfs.transform.translation.y = position_.y;
        tfs.transform.translation.z = 0.0;
        tfs.transform.rotation      = odom.pose.pose.orientation;

        tf_pub_.sendTransform(tfs);
      }

      t_prev_ = t_msg;
    }

    private:
    typename rclcpp::Subscription<twist_type>::SharedPtr twist_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::string odometry_frame_;
    std::string base_frame_;
    rclcpp::Logger logger_ctor_;
    rclcpp::Logger logger_init_cb_;
    rclcpp::Logger logger_reset_cb_;
    rclcpp::Logger logger_twist_cb_;
    tf2_ros::TransformBroadcaster tf_pub_;
    rclcpp::Time t_prev_;
    geometry_msgs::msg::Point position_;
    double yaw_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;

    static constexpr double LIN_COV{1e-6};
    static constexpr double ANG_COV{1e-4};
  };
}  // namespace ground_vehicle_twist_odometry
