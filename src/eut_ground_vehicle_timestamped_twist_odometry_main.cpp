#include <memory>
#include <stdexcept>

#include "eut_ground_vehicle_twist_odometry/eut_ground_vehicle_twist_odometry.hpp"

namespace eut_gvto = eut_ground_vehicle_twist_odometry;

int main(int argc, char** argv)
{
  constexpr bool timestamped_twist{true};

  rclcpp::init(argc, argv);

  std::shared_ptr<eut_gvto::GroundVehicleTwistOdometry<timestamped_twist>> gv_twist_odometry;

  try
  {
    gv_twist_odometry = std::make_shared<eut_gvto::GroundVehicleTwistOdometry<timestamped_twist>>(
      "ground_vehicle_twist_odometry");

    rclcpp::spin(gv_twist_odometry);
  }
  catch(std::exception& ex)
  {
    RCLCPP_FATAL(gv_twist_odometry->get_logger(), "%s.", ex.what());
  }

  rclcpp::shutdown();

  return 0;
}