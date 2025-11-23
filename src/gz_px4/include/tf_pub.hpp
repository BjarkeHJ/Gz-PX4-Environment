#ifndef TF_PUB_
#define TF_PUB_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace gz_px4 {

class PxOdomToTf : public rclcpp::Node 
{
public:
    PxOdomToTf();

private:
    void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}

#endif