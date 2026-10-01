#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include "ground_vehicle_twist_odometry/ground_vehicle_twist_odometry.hpp"

namespace
{
  using ground_vehicle_twist_odometry::GroundVehicleTwistOdometry;
  using ground_vehicle_twist_odometry::Twist2D;
  using ground_vehicle_twist_odometry::UpdateStatus;
  using ground_vehicle_twist_odometry::VelocitySample;

  constexpr std::int64_t kSecond{1000000000};
  constexpr double kPi{3.14159265358979323846};

  /**
   * @brief Create a ROS-independent velocity sample for an odometry scenario.
   *
   * @param linear_vel_x Forward velocity in the base frame, in m/s.
   * @param linear_vel_y Lateral velocity in the base frame, in m/s.
   * @param angular_vel_z Yaw velocity in rad/s.
   * @param stamp_ns Sample time in nanoseconds.
   * @return The sample accepted by the core library.
   */
  VelocitySample sample(const double linear_vel_x,
                        const double linear_vel_y,
                        const double angular_vel_z,
                        const std::int64_t stamp_ns)
  {
    // Keep unit tests independent of ROS message construction and clocks.
    return {{linear_vel_x, linear_vel_y, angular_vel_z}, stamp_ns};
  }

  /**
   * @brief Measure position error for vx(t) = t and omega(t) = 1 over one second.
   *
   * @param intervals Number of evenly spaced integration intervals.
   * @return Distance between the integrated pose and the analytic end position.
   */
  double smooth_path_error(const std::int64_t intervals)
  {
    // The analytic end position gives an independent reference for convergence.
    const double expected_x{(std::sin(1.0) + std::cos(1.0)) - 1.0};
    const double expected_y{std::sin(1.0) - std::cos(1.0)};
    GroundVehicleTwistOdometry odometry{};
    const std::int64_t step_ns{kSecond / intervals};
    odometry.update(sample(0.0, 0.0, 1.0, 0));
    for(std::int64_t index{1}; index <= intervals; ++index)
    {
      const double time{static_cast<double>(index) / static_cast<double>(intervals)};
      EXPECT_EQ(odometry.update(sample(time, 0.0, 1.0, index * step_ns)).status, UpdateStatus::Integrated);
    }
    EXPECT_NEAR(odometry.pose().yaw, 1.0, 1e-12);
    return std::hypot(odometry.pose().x - expected_x, odometry.pose().y - expected_y);
  }

  /**
   * @brief Check that an invalid velocity cannot replace the accepted time baseline.
   *
   * @param odometry Integrator holding the valid sample at one second.
   * @param twist Sample that must be rejected at two seconds.
   */
  void expect_rejected_velocity_preserves_time(GroundVehicleTwistOdometry& odometry, const Twist2D& twist)
  {
    // Every invalid component must leave the same last accepted sample available.
    EXPECT_EQ(odometry.update({twist, 2 * kSecond}).status, UpdateStatus::InvalidVelocity);
    const auto retained{odometry.last_sample()};
    ASSERT_TRUE(retained.has_value());
    EXPECT_EQ(retained.value_or(VelocitySample{}).stamp_ns, kSecond);
  }

  /**
   * @brief Confirm that two samples establish one interval integrated with their mean velocity.
   */
  TEST(GroundVehicleTwistOdometryTest, InitializesAndIntegratesTheAverageTwist)
  {
    // The first sample sets the time origin but cannot describe a previous interval.
    GroundVehicleTwistOdometry odometry{};
    EXPECT_FALSE(odometry.last_sample().has_value());
    EXPECT_EQ(odometry.update(sample(0.0, 0.0, 0.0, 0)).status, UpdateStatus::Initialized);
    EXPECT_DOUBLE_EQ(odometry.pose().x, 0.0);

    const auto result{odometry.update(sample(1.0, 0.0, 0.0, kSecond))};
    EXPECT_EQ(result.status, UpdateStatus::Integrated);
    EXPECT_DOUBLE_EQ(result.dt_seconds, 1.0);
    EXPECT_DOUBLE_EQ(odometry.pose().x, 0.5);
    const auto retained{odometry.last_sample()};
    ASSERT_TRUE(retained.has_value());
    EXPECT_DOUBLE_EQ(retained.value_or(VelocitySample{}).twist.vx, 1.0);
  }

  /**
   * @brief Check an analytic curved path with both forward and lateral velocity.
   */
  TEST(GroundVehicleTwistOdometryTest, IntegratesConstantBodyTwistOnAPlanarArc)
  {
    // With constant body velocity, the closed-form arc is an independent expected result.
    constexpr double kLateralSpeed{2.0};
    constexpr double kQuarterTurn{kPi / 2.0};
    GroundVehicleTwistOdometry odometry{};
    const auto twist{sample(1.0, kLateralSpeed, kQuarterTurn, 0)};
    odometry.update(twist);
    odometry.update(sample(1.0, kLateralSpeed, kQuarterTurn, kSecond));

    EXPECT_NEAR(odometry.pose().x, -(kLateralSpeed / kPi), 1e-12);
    EXPECT_NEAR(odometry.pose().y, 6.0 / kPi, 1e-12);
    EXPECT_NEAR(odometry.pose().yaw, kQuarterTurn, 1e-12);

    // Pure clockwise rotation must change orientation without translating the base origin.
    odometry.reset();
    odometry.update(sample(0.0, 0.0, -kQuarterTurn, 0));
    odometry.update(sample(0.0, 0.0, -kQuarterTurn, kSecond));
    EXPECT_DOUBLE_EQ(odometry.pose().x, 0.0);
    EXPECT_DOUBLE_EQ(odometry.pose().y, 0.0);
    EXPECT_NEAR(odometry.pose().yaw, -kQuarterTurn, 1e-12);
  }

  /**
   * @brief Confirm second-order convergence against a smooth path with a known solution.
   */
  TEST(GroundVehicleTwistOdometryTest, KeepsSecondOrderAccuracyForSmoothSamples)
  {
    // Halving the interval should reduce a second-order position error by about four.
    const double error_50{smooth_path_error(50)};
    const double error_100{smooth_path_error(100)};
    EXPECT_LT(error_50, 1e-4);
    EXPECT_GT(error_50 / error_100, 3.5);
    EXPECT_LT(error_50 / error_100, 4.5);
  }

  /**
   * @brief Confirm equal-time samples update the held velocity without moving the pose.
   */
  TEST(GroundVehicleTwistOdometryTest, ReplacesASameTimeSampleWithoutIntegrating)
  {
    constexpr double kReplacementSpeed{3.0};
    GroundVehicleTwistOdometry odometry{};
    odometry.update(sample(1.0, 0.0, 0.0, 0));
    const auto result{odometry.update(sample(kReplacementSpeed, 0.0, 0.0, 0))};
    EXPECT_EQ(result.status, UpdateStatus::SameTimestamp);
    EXPECT_DOUBLE_EQ(odometry.pose().x, 0.0);
    EXPECT_DOUBLE_EQ(odometry.last_sample().value_or(VelocitySample{}).twist.vx, kReplacementSpeed);

    // The following interval must use the replacement as its starting sample.
    odometry.update(sample(kReplacementSpeed, 0.0, 0.0, kSecond));
    EXPECT_DOUBLE_EQ(odometry.pose().x, kReplacementSpeed);
  }

  /**
   * @brief Confirm rejected samples cannot change the retained pose or time baseline.
   */
  TEST(GroundVehicleTwistOdometryTest, RejectsOldAndNonFiniteSamplesWithoutLosingTime)
  {
    GroundVehicleTwistOdometry odometry{};
    odometry.update(sample(1.0, 0.0, 0.0, kSecond));
    EXPECT_EQ(odometry.update(sample(9.0, 0.0, 0.0, 0)).status, UpdateStatus::BackwardsTime);

    // A later valid sample still closes the interval from the last accepted sample.
    for(const double invalid: {std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::infinity(),
                               -std::numeric_limits<double>::infinity()})
    {
      expect_rejected_velocity_preserves_time(odometry, {invalid, 0.0, 0.0});
      expect_rejected_velocity_preserves_time(odometry, {0.0, invalid, 0.0});
      expect_rejected_velocity_preserves_time(odometry, {0.0, 0.0, invalid});
    }

    EXPECT_EQ(odometry.update(sample(1.0, 0.0, 0.0, 3 * kSecond)).status, UpdateStatus::Integrated);
    EXPECT_DOUBLE_EQ(odometry.pose().x, 2.0);
  }

  /**
   * @brief Integrate a long valid interval and reject arithmetic overflow atomically.
   */
  TEST(GroundVehicleTwistOdometryTest, IntegratesLongIntervalsAndRejectsNonFiniteResults)
  {
    // The operational rate warning belongs to ROS; the core integrates every positive interval.
    GroundVehicleTwistOdometry odometry{};
    odometry.update(sample(1.0, 0.0, 0.0, 0));
    EXPECT_EQ(odometry.update(sample(3.0, 0.0, 0.0, 10 * kSecond)).status, UpdateStatus::Integrated);
    EXPECT_DOUBLE_EQ(odometry.pose().x, 20.0);

    // Finite inputs can still overflow during integration, so the old pose must remain intact.
    odometry.reset();
    const double maximum{std::numeric_limits<double>::max()};
    odometry.update(sample(maximum, 0.0, 0.0, 0));
    const auto result{odometry.update(sample(maximum, 0.0, 0.0, 2 * kSecond))};
    EXPECT_EQ(result.status, UpdateStatus::NumericalError);
    EXPECT_DOUBLE_EQ(odometry.pose().x, 0.0);
    ASSERT_TRUE(odometry.last_sample().has_value());
    EXPECT_EQ(odometry.last_sample().value_or(VelocitySample{}).stamp_ns, 0);
  }

  /**
   * @brief Preserve interval arithmetic across the signed timestamp range and after reset.
   */
  TEST(GroundVehicleTwistOdometryTest, HandlesSignedTimestampExtremesAndReset)
  {
    // The integer difference is computed before conversion to floating-point seconds.
    GroundVehicleTwistOdometry odometry{};
    odometry.update(sample(1.0, 0.0, 0.0, std::numeric_limits<std::int64_t>::min()));
    const auto result{odometry.update(sample(1.0, 0.0, 0.0, std::numeric_limits<std::int64_t>::max()))};
    EXPECT_EQ(result.status, UpdateStatus::Integrated);
    EXPECT_NEAR(odometry.pose().x, result.dt_seconds, 1e-5);

    // Reset also removes the previous timestamp, so the next sample only initializes.
    odometry.reset();
    odometry.reset();
    EXPECT_FALSE(odometry.last_sample().has_value());
    EXPECT_DOUBLE_EQ(odometry.pose().x, 0.0);
    EXPECT_EQ(odometry.update(sample(2.0, 0.0, 0.0, 100)).status, UpdateStatus::Initialized);
  }
}  // namespace
