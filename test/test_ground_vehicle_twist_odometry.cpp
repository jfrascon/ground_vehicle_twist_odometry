#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <memory>
#include <thread>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "ground_vehicle_twist_odometry/ground_vehicle_twist_odometry.hpp"

namespace
{
  using OdometryNode = ground_vehicle_twist_odometry::GroundVehicleTwistOdometry<true>;

  class GroundVehicleTwistOdometryTest: public ::testing::Test
  {
    protected:
      static void SetUpTestSuite()
      {
        rclcpp::init(0, nullptr);
      }

      static void TearDownTestSuite()
      {
        rclcpp::shutdown();
      }
  };

  TEST_F(GroundVehicleTwistOdometryTest, integrates_from_zero_initialized_pose)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter{"odometry_frame", "test_odom"},
      rclcpp::Parameter{"base_frame", "test_base_link"},
      rclcpp::Parameter{"publish_tf", false},
      rclcpp::Parameter{"expected_incoming_twist_msg_rate", 1.0},
    });
    options.arguments({"--ros-args", "--remap", "__ns:=/ground_vehicle_twist_odometry_test"});

    auto odometry_node = std::make_shared<OdometryNode>("ground_vehicle_twist_odometry", options);
    auto observer = std::make_shared<rclcpp::Node>("odometry_observer", "/ground_vehicle_twist_odometry_test");

    std::promise<nav_msgs::msg::Odometry> odometry_promise;
    auto odometry_future = odometry_promise.get_future();
    bool received_odometry{false};
    auto subscription = observer
                          ->create_subscription<nav_msgs::msg::Odometry>("odom",
                                                                         10,
                                                                         [&odometry_promise, &received_odometry](
                                                                           const nav_msgs::msg::Odometry& message) {
                                                                           if(!received_odometry)
                                                                           {
                                                                             received_odometry = true;
                                                                             odometry_promise.set_value(message);
                                                                           }
                                                                         });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(observer);
    std::thread spin_thread{[&executor]() {
      executor.spin();
    }};

    for(int attempt = 0; attempt < 100 && subscription->get_publisher_count() == 0; ++attempt)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }

    auto initial_twist = std::make_shared<geometry_msgs::msg::TwistStamped>();
    initial_twist->header.stamp.sec = 1;
    initial_twist->header.stamp.nanosec = 0;
    odometry_node->init_cb(initial_twist);

    auto next_twist = std::make_shared<geometry_msgs::msg::TwistStamped>();
    next_twist->header.stamp.sec = 2;
    next_twist->header.stamp.nanosec = 0;
    next_twist->twist.linear.x = 1.0;
    odometry_node->twist_cb(next_twist);

    const auto result_status = odometry_future.wait_for(std::chrono::seconds{2});

    executor.cancel();
    spin_thread.join();

    ASSERT_EQ(result_status, std::future_status::ready);
    const auto odometry = odometry_future.get();
    EXPECT_DOUBLE_EQ(odometry.pose.pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(odometry.pose.pose.position.y, 0.0);
    EXPECT_DOUBLE_EQ(odometry.pose.pose.orientation.z, 0.0);
    EXPECT_DOUBLE_EQ(odometry.pose.pose.orientation.w, 1.0);
  }
}  // namespace
