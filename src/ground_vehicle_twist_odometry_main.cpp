#include "ground_vehicle_twist_odometry/ground_vehicle_twist_odometry.hpp"

#include <memory>
#include <exception>

#include <rclcpp/rclcpp.hpp>

namespace gvto = ground_vehicle_twist_odometry;

#if defined(USE_TIMESTAMPED_TWIST) && USE_TIMESTAMPED_TWIST
using GVTO = gvto::GroundVehicleTwistOdometry<true>;
#else
using GVTO = gvto::GroundVehicleTwistOdometry<false>;
#endif

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<GVTO> node;
  int ret{0};

  try
  {
    node = std::make_shared<GVTO>("ground_vehicle_twist_odometry");
    rclcpp::spin(node);
  }
  catch(const std::exception& ex)
  {
    const auto logger = node ? node->get_logger() :
                               rclcpp::get_logger("ground_vehicle_twist_odometry");
    RCLCPP_FATAL(logger, "%s.", ex.what());
    ret = 1;
  }

  rclcpp::shutdown();
  return ret;
}
