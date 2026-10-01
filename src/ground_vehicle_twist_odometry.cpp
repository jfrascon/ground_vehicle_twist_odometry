#include "ground_vehicle_twist_odometry/ground_vehicle_twist_odometry.hpp"

#include <cmath>
#include <cstdint>
#include <optional>

namespace ground_vehicle_twist_odometry
{
  namespace
  {
    constexpr double kNanosecondsPerSecond{1e9};
    constexpr double kSincTaylorThreshold{1e-4};
    constexpr double kSincQuadraticDenominator{6.0};
    constexpr double kSincQuarticDenominator{120.0};

    /**
     * @brief Compute sin(angle)/angle without dividing by zero for a straight motion step.
     *
     * @param half_rotation Half of the rotation during one sample interval, in radians.
     * @return The dimensionless factor that corrects displacement along a curved path.
     */
    double sinc(const double half_rotation)
    {
      if(std::abs(half_rotation) < kSincTaylorThreshold)
      {
        // The Taylor series includes sinc(0) = 1 and preserves the correction for small turns.
        const double rotation_squared{half_rotation * half_rotation};
        return 1.0 - (rotation_squared / kSincQuadraticDenominator) +
               ((rotation_squared * rotation_squared) / kSincQuarticDenominator);
      }
      return std::sin(half_rotation) / half_rotation;
    }

    /**
     * @brief Check whether every velocity used by the planar integrator is finite.
     *
     * @param twist Velocity of the base origin in the base frame.
     * @return True when the sample can safely enter the odometry state.
     */
    bool is_finite(const Twist2D& twist)
    {
      // Retaining a non-finite velocity would corrupt the next interval as well.
      return std::isfinite(twist.vx) && std::isfinite(twist.vy) && std::isfinite(twist.omega);
    }

    /**
     * @brief Check a complete candidate pose before replacing the previous valid pose.
     *
     * @param pose Pose computed for the new sample time.
     * @return True when future integrations can start from this pose.
     */
    bool is_finite(const Pose2D& pose)
    {
      // A non-finite accumulated pose would propagate into all later updates.
      return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
    }
  }  // namespace

  /**
   * @brief Integrate the interval closed by a new instantaneous velocity sample.
   *
   * @param sample Planar base velocity and its time in nanoseconds.
   * @return The update outcome and the interval in seconds when integration was attempted.
   */
  UpdateResult GroundVehicleTwistOdometry::update(const VelocitySample& sample)
  {
    // Validate first so rejected samples leave both the pose and time baseline unchanged.
    if(!is_finite(sample.twist))
    {
      return {UpdateStatus::InvalidVelocity};
    }

    if(!last_sample_)
    {
      // The first sample defines the odometry origin; it closes no interval.
      last_sample_ = sample;
      return {UpdateStatus::Initialized};
    }

    if(sample.stamp_ns < last_sample_->stamp_ns)
    {
      return {UpdateStatus::BackwardsTime};
    }

    if(sample.stamp_ns == last_sample_->stamp_ns)
    {
      // No time has elapsed, but the latest velocity will represent this instant going forward.
      last_sample_ = sample;
      return {UpdateStatus::SameTimestamp};
    }

    // Unsigned subtraction avoids overflow when valid timestamps span the signed range.
    const auto dt_ns{static_cast<std::uint64_t>(sample.stamp_ns) - static_cast<std::uint64_t>(last_sample_->stamp_ns)};
    const double interval_seconds{static_cast<double>(dt_ns) / kNanosecondsPerSecond};

    // Approximate motion between instantaneous samples by their average body velocity.
    const Twist2D average{
      (0.5 * last_sample_->twist.vx) + (0.5 * sample.twist.vx),
      (0.5 * last_sample_->twist.vy) + (0.5 * sample.twist.vy),
      (0.5 * last_sample_->twist.omega) + (0.5 * sample.twist.omega),
    };

    const double delta_yaw{average.omega * interval_seconds};
    const double half_yaw{0.5 * delta_yaw};
    const double middle_yaw{pose_.yaw + half_yaw};
    if(!is_finite(average) || !std::isfinite(delta_yaw) || !std::isfinite(middle_yaw))
    {
      return {UpdateStatus::NumericalError, interval_seconds};
    }

    // The midpoint orientation gives the displacement direction; sinc accounts for its arc.
    const double distance_scale{interval_seconds * sinc(half_yaw)};
    const double cosine{std::cos(middle_yaw)};
    const double sine{std::sin(middle_yaw)};
    const double delta_x{distance_scale * ((average.vx * cosine) - (average.vy * sine))};
    const double delta_y{distance_scale * ((average.vx * sine) + (average.vy * cosine))};
    const Pose2D candidate{pose_.x + delta_x, pose_.y + delta_y, pose_.yaw + delta_yaw};
    if(!std::isfinite(distance_scale) || !is_finite(candidate))
    {
      return {UpdateStatus::NumericalError, interval_seconds};
    }

    // Commit pose and sample together only after every candidate value has passed validation.
    pose_ = candidate;
    last_sample_ = sample;
    return {UpdateStatus::Integrated, interval_seconds};
  }

  /**
   * @brief Start a new odometry session at pose zero without a retained velocity sample.
   */
  void GroundVehicleTwistOdometry::reset() noexcept
  {
    // The next accepted sample establishes a new time baseline.
    // It does not integrate from the previous session.
    pose_ = Pose2D{};
    last_sample_.reset();
  }

  /**
   * @brief Read the current planar pose relative to the latest odometry origin.
   *
   * @return A value copy of the accumulated pose.
   */
  Pose2D GroundVehicleTwistOdometry::pose() const noexcept
  {
    // Return a copy so callers cannot mutate the integrator's internal pose.
    return pose_;
  }

  /**
   * @brief Read the last accepted velocity sample and its time, if one exists.
   *
   * @return A value copy of the retained sample, or an empty optional before initialization.
   */
  std::optional<VelocitySample> GroundVehicleTwistOdometry::last_sample() const noexcept
  {
    // The optional is empty after construction and reset.
    return last_sample_;
  }
}  // namespace ground_vehicle_twist_odometry
