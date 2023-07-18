#include <memory>
#include <stdexcept>

#include "eut_ground_vehicle_twist_odometry/eut_ground_vehicle_twist_odometry.hpp"

namespace eut_gvto = eut_ground_vehicle_twist_odometry;

int main(int argc, char** argv)
{
  constexpr bool twist_is_timestamped{false};

  rclcpp::init(argc, argv);

  std::shared_ptr<eut_gvto::GroundVehicleTwistOdometry<twist_is_timestamped>> gvto;

  try
  {
    gvto = std::make_shared<eut_gvto::GroundVehicleTwistOdometry<twist_is_timestamped>>();
    rclcpp::spin(gvto);
  }
  catch(std::exception& ex)
  {
    RCLCPP_FATAL(gvto->get_logger(), "%s.", ex.what());
  }

  rclcpp::shutdown();

  return 0;
}