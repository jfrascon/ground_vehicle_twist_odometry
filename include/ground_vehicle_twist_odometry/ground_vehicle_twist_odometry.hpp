#pragma once

#include <cstdint>
#include <optional>

namespace ground_vehicle_twist_odometry
{
  // Velocities of the base origin, expressed in the base frame.
  struct Twist2D
  {
      double vx{0.0};
      double vy{0.0};
      double omega{0.0};
  };

  // Planar pose of the base origin relative to the current odometry origin.
  struct Pose2D
  {
      double x{0.0};
      double y{0.0};
      double yaw{0.0};
  };

  // All samples passed to one instance must use the same clock and time origin.
  struct VelocitySample
  {
      Twist2D twist{};
      std::int64_t stamp_ns{0};
  };

  enum class UpdateStatus
  {
    Initialized,
    Integrated,
    SameTimestamp,
    InvalidVelocity,
    BackwardsTime,
    NumericalError,
  };

  struct UpdateResult
  {
      UpdateStatus status;
      // Positive only when integration was attempted.
      double dt_seconds{0.0};
  };

  // Call update, reset, and state getters serially from each consumer.
  class GroundVehicleTwistOdometry
  {
    public:
      UpdateResult update(const VelocitySample& sample);
      void reset() noexcept;

      Pose2D pose() const noexcept;
      std::optional<VelocitySample> last_sample() const noexcept;

    private:
      Pose2D pose_{};
      std::optional<VelocitySample> last_sample_{};
  };
}  // namespace ground_vehicle_twist_odometry
