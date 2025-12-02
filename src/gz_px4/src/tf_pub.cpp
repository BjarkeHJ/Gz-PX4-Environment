/*

This transform publisher takes the odometry date from PX4 Sitl
and creates the transform odom -> base_link transform. 
This transform is thus dynamically updated by reading pose estimation of the simulated IMU-odometry data
"Transform from where it started to where it think it is..."

*/

#include "tf_pub.hpp"

namespace gz_px4
{
 
PxOdomToTf::PxOdomToTf() : Node("px4_odom_to_tf") 
{
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    auto qos_profile = rclcpp::SensorDataQoS();
    odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry",
        qos_profile,
        std::bind(&PxOdomToTf::odomCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "px4_odom_to_tf node started");
}

void PxOdomToTf::odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    using namespace px4_ros_com::frame_transforms;

    // --- Position: NED -> ENU ---
    // PX4: position[0]=N, [1]=E, [2]=D  (NED)
    Eigen::Vector3d ned_pos(msg->position[0],
                            msg->position[1],
                            msg->position[2]);

    // ROS: x=E, y=N, z=U  (ENU)
    Eigen::Vector3d enu_pos = ned_to_enu_local_frame(ned_pos);

    // --- Orientation: aircraft(FRD)->NED  ->  base_link(FLU)->ENU ---

    // PX4 stores quaternion as Hamilton (w, x, y, z) from body(FRD) -> world(NED)
    Eigen::Quaterniond q_px4 = utils::quaternion::array_to_eigen_quat(msg->q);  // (w,x,y,z)

    // Convert aircraft(FRD) frame to ROS base_link(FLU) frame
    Eigen::Quaterniond q_baselink_ned = aircraft_to_baselink_orientation(q_px4);

    // Convert world frame NED -> ENU
    Eigen::Quaterniond q_baselink_enu = ned_to_enu_orientation(q_baselink_ned);

    // --- Pack into TF message ---
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = "odom";      // ENU odom 
    tf_msg.child_frame_id  = "base_link"; // FLU body

    tf_msg.transform.translation.x = enu_pos.x();
    tf_msg.transform.translation.y = enu_pos.y();
    tf_msg.transform.translation.z = enu_pos.z();

    tf_msg.transform.rotation.w = q_baselink_enu.w();
    tf_msg.transform.rotation.x = q_baselink_enu.x();
    tf_msg.transform.rotation.y = q_baselink_enu.y();
    tf_msg.transform.rotation.z = q_baselink_enu.z();

    tf_broadcaster_->sendTransform(tf_msg);
}

}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gz_px4::PxOdomToTf>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}