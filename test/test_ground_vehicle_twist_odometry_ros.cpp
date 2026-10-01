#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <future>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/utilities.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "ground_vehicle_twist_odometry/ground_vehicle_twist_odometry_ros.hpp"

namespace
{
  using ground_vehicle_twist_odometry::GroundVehicleTwistOdometryROS;

  constexpr std::size_t kQueueDepth{10};
  constexpr std::chrono::milliseconds kSpinTimeout{2000};
  constexpr std::chrono::milliseconds kSpinPollPeriod{5};
  constexpr std::chrono::milliseconds kQuietPeriod{50};

  /**
   * @brief Run ready ROS callbacks until an observed condition or a bounded timeout.
   *
   * @tparam Predicate A callable that checks the expected ROS observation.
   * @param executor Executor containing the wrapper and observer nodes.
   * @param predicate Condition checked after each executor pass.
   * @param timeout Maximum time to wait for the condition.
   * @return True if the condition became observable before the timeout.
   */
  template<typename Predicate>
  bool spin_until(rclcpp::executors::SingleThreadedExecutor& executor,
                  Predicate predicate,
                  const std::chrono::milliseconds timeout = kSpinTimeout)
  {
    // Spinning both nodes lets the test observe the real topic and service callbacks.
    const auto deadline{std::chrono::steady_clock::now() + timeout};
    while(std::chrono::steady_clock::now() < deadline)
    {
      executor.spin_some();
      if(predicate())
      {
        return true;
      }
      std::this_thread::sleep_for(kSpinPollPeriod);
    }
    executor.spin_some();
    return predicate();
  }

  /** @brief Initialize one ROS context for the wrapper integration tests. */
  class GroundVehicleTwistOdometryROSTest: public ::testing::Test
  {
    protected:
      /**
       * @brief Start ROS once before tests create publishers, subscribers, and clients.
       */
      static void SetUpTestSuite()
      {
        // The same context is shared by both message-type variants in this suite.
        rclcpp::init(0, nullptr);
      }

      /**
       * @brief Release the shared ROS context after all wrapper tests finish.
       */
      static void TearDownTestSuite()
      {
        // Shutdown follows destruction of each test's nodes and executor.
        rclcpp::shutdown();
      }
  };

  /**
   * @brief Exercise stamped topic input, TF publication, and reset through real ROS interfaces.
   */
  // ROS and GTest macros expand into branches outside this test's scenario logic.
  // NOLINTNEXTLINE
  TEST_F(GroundVehicleTwistOdometryROSTest, StampedInputPublishesCurrentTwistTfAndResets)
  {
    // Parameter overrides give the test frames that cannot be confused with package defaults.
    rclcpp::NodeOptions options{};
    options.parameter_overrides({
      rclcpp::Parameter{"odometry_frame", "test_odom"},
      rclcpp::Parameter{"base_frame", "test_base_link"},
      rclcpp::Parameter{"expected_incoming_twist_msg_rate", 1.0},
    });
    options.arguments({"--ros-args", "--remap", "__ns:=/gvto_stamped_test"});
    auto odometry_node{std::make_shared<GroundVehicleTwistOdometryROS<true>>("ground_vehicle_twist_odometry", options)};
    auto observer{std::make_shared<rclcpp::Node>("observer", "/gvto_stamped_test")};

    std::vector<nav_msgs::msg::Odometry> odometry_messages{};
    std::vector<tf2_msgs::msg::TFMessage> tf_messages{};
    auto odom_sub{observer->create_subscription<nav_msgs::msg::Odometry>("odom",
                                                                         kQueueDepth,
                                                                         [&](const nav_msgs::msg::Odometry& msg) {
                                                                           odometry_messages.push_back(msg);
                                                                         })};
    auto tf_sub{observer->create_subscription<tf2_msgs::msg::TFMessage>("/tf",
                                                                        kQueueDepth,
                                                                        [&](const tf2_msgs::msg::TFMessage& msg) {
                                                                          tf_messages.push_back(msg);
                                                                        })};
    auto twist_pub{observer->create_publisher<geometry_msgs::msg::TwistStamped>("twist", kQueueDepth)};
    auto reset_client{observer->create_client<std_srvs::srv::Empty>("reset_odom")};

    // Wait for discovery before sending the first sample, which establishes the time origin.
    rclcpp::executors::SingleThreadedExecutor executor{};
    executor.add_node(odometry_node);
    executor.add_node(observer);
    ASSERT_TRUE(spin_until(executor, [&]() {
      return twist_pub->get_subscription_count() > 0 && odom_sub->get_publisher_count() > 0 &&
             tf_sub->get_publisher_count() > 0 && reset_client->service_is_ready();
    }));

    geometry_msgs::msg::TwistStamped first{};
    first.header.stamp.sec = 0;
    twist_pub->publish(first);
    ASSERT_FALSE(spin_until(
      executor,
      [&]() {
        return !odometry_messages.empty();
      },
      kQuietPeriod));

    // The pose uses the interval mean, while Odometry.twist must contain the second sample.
    geometry_msgs::msg::TwistStamped second{};
    second.header.stamp.sec = 1;
    second.twist.linear.x = 1.0;
    constexpr double kUnintegratedLinearZ{0.25};
    second.twist.linear.z = kUnintegratedLinearZ;
    twist_pub->publish(second);
    ASSERT_TRUE(spin_until(executor, [&]() {
      return odometry_messages.size() == 1 && !tf_messages.empty();
    }));

    const auto& odom{odometry_messages.front()};
    EXPECT_EQ(odom.header.stamp.sec, 1);
    EXPECT_EQ(odom.header.frame_id, "test_odom");
    EXPECT_EQ(odom.child_frame_id, "test_base_link");
    EXPECT_DOUBLE_EQ(odom.pose.pose.position.x, 0.5);
    EXPECT_DOUBLE_EQ(odom.twist.twist.linear.x, 1.0);
    EXPECT_DOUBLE_EQ(odom.twist.twist.linear.z, kUnintegratedLinearZ);
    EXPECT_DOUBLE_EQ(odom.pose.covariance[0], 1e-6);
    EXPECT_DOUBLE_EQ(odom.twist.covariance[0], 5e-7);
    EXPECT_EQ(tf_messages.front().transforms.front().header.stamp.sec, 1);
    EXPECT_EQ(tf_messages.front().transforms.front().child_frame_id, "test_base_link");

    // Reset must clear the integrator while preserving the same ROS subscription.
    const auto request{std::make_shared<std_srvs::srv::Empty::Request>()};
    auto reset_future{reset_client->async_send_request(request)};
    ASSERT_TRUE(spin_until(executor, [&]() {
      return reset_future.wait_for(std::chrono::seconds{0}) == std::future_status::ready;
    }));

    geometry_msgs::msg::TwistStamped after_reset{};
    after_reset.header.stamp.sec = 2;
    constexpr double kPostResetSpeed{2.0};
    after_reset.twist.linear.x = kPostResetSpeed;
    twist_pub->publish(after_reset);
    ASSERT_FALSE(spin_until(
      executor,
      [&]() {
        return odometry_messages.size() > 1;
      },
      kQuietPeriod));

    after_reset.header.stamp.sec = 3;
    twist_pub->publish(after_reset);
    ASSERT_TRUE(spin_until(executor, [&]() {
      return odometry_messages.size() == 2;
    }));
    EXPECT_DOUBLE_EQ(odometry_messages.back().pose.pose.position.x, kPostResetSpeed);
    EXPECT_EQ(twist_pub->get_subscription_count(), 1U);
  }

  /**
   * @brief Verify plain Twist reception time and rejection of an invalid published component.
   */
  // ROS and GTest macros expand into branches outside this test's scenario logic.
  // NOLINTNEXTLINE
  TEST_F(GroundVehicleTwistOdometryROSTest, PlainTwistUsesNodeTimeAndRejectsInvalidMessages)
  {
    // Plain Twist has no header, so the wrapper must timestamp it from its node clock.
    rclcpp::NodeOptions options{};
    options.parameter_overrides({
      rclcpp::Parameter{"publish_tf", false},
      rclcpp::Parameter{"expected_incoming_twist_msg_rate", 1.0},
    });
    options.arguments({"--ros-args", "--remap", "__ns:=/gvto_plain_test"});
    auto odometry_node{
      std::make_shared<GroundVehicleTwistOdometryROS<false>>("ground_vehicle_twist_odometry", options)};
    auto observer{std::make_shared<rclcpp::Node>("observer", "/gvto_plain_test")};

    std::vector<nav_msgs::msg::Odometry> odometry_messages{};
    auto odom_sub{observer->create_subscription<nav_msgs::msg::Odometry>("odom",
                                                                         kQueueDepth,
                                                                         [&](const nav_msgs::msg::Odometry& msg) {
                                                                           odometry_messages.push_back(msg);
                                                                         })};
    auto twist_pub{observer->create_publisher<geometry_msgs::msg::Twist>("twist", kQueueDepth)};

    rclcpp::executors::SingleThreadedExecutor executor{};
    executor.add_node(odometry_node);
    executor.add_node(observer);
    ASSERT_TRUE(spin_until(executor, [&]() {
      return twist_pub->get_subscription_count() > 0 && odom_sub->get_publisher_count() > 0;
    }));

    // The first callback has no interval to publish; the second closes one measured interval.
    const auto before{observer->now()};
    geometry_msgs::msg::Twist twist{};
    twist.linear.x = 1.0;
    twist_pub->publish(twist);
    ASSERT_FALSE(spin_until(
      executor,
      [&]() {
        return !odometry_messages.empty();
      },
      std::chrono::milliseconds{40}));

    twist_pub->publish(twist);
    ASSERT_TRUE(spin_until(executor, [&]() {
      return odometry_messages.size() == 1;
    }));
    const auto& odom{odometry_messages.front()};
    EXPECT_GE(rclcpp::Time(odom.header.stamp).nanoseconds(), before.nanoseconds());
    EXPECT_LE(rclcpp::Time(odom.header.stamp).nanoseconds(), observer->now().nanoseconds());
    EXPECT_GT(odom.pose.pose.position.x, 0.0);
    EXPECT_DOUBLE_EQ(odom.twist.twist.linear.x, 1.0);

    // Even unused planar components must be finite because the full Twist is published.
    twist.linear.z = std::numeric_limits<double>::quiet_NaN();
    twist_pub->publish(twist);
    ASSERT_FALSE(spin_until(
      executor,
      [&]() {
        return odometry_messages.size() > 1;
      },
      kQuietPeriod));
  }
}  // namespace
