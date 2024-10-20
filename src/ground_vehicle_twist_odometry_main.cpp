#include <memory>
#include <stdexcept>

#include "eut_ground_vehicle_twist_odometry/ground_vehicle_twist_odometry.hpp"

namespace gvto = ground_vehicle_twist_odometry;

int main(int argc, char** argv)
{
  constexpr bool timestamped_twist{false};

  rclcpp::init(argc, argv);

  std::shared_ptr<gvto::GroundVehicleTwistOdometry<timestamped_twist>> gv_twist_odometry;

  try
  {
    gv_twist_odometry = std::make_shared<gvto::GroundVehicleTwistOdometry<timestamped_twist>>("ground_vehicle_twist_odometry");

    rclcpp::spin(gv_twist_odometry);
  }
  catch(std::exception& ex)
  {
    RCLCPP_FATAL(gv_twist_odometry->get_logger(), "%s.", ex.what());
  }

  rclcpp::shutdown();

  return 0;
}