#include "ground_vehicle_twist_odometry/ground_vehicle_twist_odometry_ros.hpp"

#include <exception>
#include <memory>

#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>

namespace gvto = ground_vehicle_twist_odometry;

#if defined(USE_TIMESTAMPED_TWIST) && USE_TIMESTAMPED_TWIST
using GVTO = gvto::GroundVehicleTwistOdometryROS<true>;
#else
using GVTO = gvto::GroundVehicleTwistOdometryROS<false>;
#endif

/**
 * @brief Run the ROS wrapper selected when this executable was built.
 *
 * @param argc Number of command-line arguments passed to ROS.
 * @param argv Command-line arguments passed to ROS.
 * @return Zero after a normal shutdown, or one if construction or spinning fails.
 */
// ROS lifecycle functions expand into branches that are outside this entry point's decisions.
// NOLINTNEXTLINE
int main(int argc, char** argv)
{
  // The two executables differ only in their input message type; both use the same wrapper code.
  rclcpp::init(argc, argv);
  std::shared_ptr<GVTO> node{};
  int ret{0};

  try
  {
    // Keep the node pointer available so failures can be logged under its ROS name.
    node = std::make_shared<GVTO>("ground_vehicle_twist_odometry");
    rclcpp::spin(node);
  }
  catch(const std::exception& ex)
  {
    const auto logger{node ? node->get_logger() : rclcpp::get_logger("ground_vehicle_twist_odometry")};
    RCLCPP_FATAL(logger, "%s.", ex.what());
    ret = 1;
  }

  // Release the ROS context after either a normal stop or a reported failure.
  rclcpp::shutdown();
  return ret;
}
