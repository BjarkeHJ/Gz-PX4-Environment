#ifndef TF_PUB_
#define TF_PUB_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "px4_ros_com/frame_transforms.h"

namespace gz_px4 {

class PxOdomToTf : public rclcpp::Node 
{
public:
    PxOdomToTf();

private:
    void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void odomCallback_gt(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_gt_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_gt_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    px4_msgs::msg::VehicleOdometry::SharedPtr last_px4_odom_msg_;
    rclcpp::Time last_px4_odom_rx_time_;
};

}

#endif