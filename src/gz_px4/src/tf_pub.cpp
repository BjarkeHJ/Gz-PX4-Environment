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

    /* Odometry estimate from PX4 Flight Stack */
    auto qos_profile = rclcpp::SensorDataQoS();
    odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry",
        qos_profile,
        std::bind(&PxOdomToTf::odomCallback, this, std::placeholders::_1)
    );

    /* Ground Truth Odometry from GZ Sim */
    odom_gt_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/x500/odom",
        qos_profile,
        std::bind(&PxOdomToTf::odomCallback_gt, this, std::placeholders::_1)
    );

    odom_gt_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", qos_profile);

    RCLCPP_INFO(this->get_logger(), "px4_odom_to_tf node started");
}

void PxOdomToTf::odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    using namespace px4_ros_com::frame_transforms;
    last_px4_odom_msg_ = msg;
    last_px4_odom_rx_time_ = this->get_clock()->now();

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
    tf_msg.header.frame_id = "odom";      // FLU odom (starting fixed frame) 
    tf_msg.child_frame_id  = "base_link"; // FLU base_link

    tf_msg.transform.translation.x = enu_pos.x();
    tf_msg.transform.translation.y = enu_pos.y();
    tf_msg.transform.translation.z = enu_pos.z();

    tf_msg.transform.rotation.w = q_baselink_enu.w();
    tf_msg.transform.rotation.x = q_baselink_enu.x();
    tf_msg.transform.rotation.y = q_baselink_enu.y();
    tf_msg.transform.rotation.z = q_baselink_enu.z();

    tf_broadcaster_->sendTransform(tf_msg);
}

void PxOdomToTf::odomCallback_gt(const nav_msgs::msg::Odometry::SharedPtr msg) {
    using namespace px4_ros_com::frame_transforms;
    
    if (!last_px4_odom_msg_) return;
    const rclcpp::Time now_ros = this->get_clock()->now();
    const int64_t dt_us = (now_ros - last_px4_odom_rx_time_).nanoseconds() / 1000;
    const uint64_t px4_now_us = static_cast<uint64_t>(static_cast<int64_t>(last_px4_odom_msg_->timestamp) + dt_us);

    const Eigen::Vector3d enu_p(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z
    );
    const Eigen::Vector3d ned_p = enu_to_ned_local_frame(enu_p);

    const Eigen::Quaterniond q_flu_enu(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
    );
    const Eigen::Quaterniond q_flu_ned = enu_to_ned_orientation(q_flu_enu);
    const Eigen::Quaterniond q_frd_ned = baselink_to_aircraft_orientation(q_flu_ned);

    px4_msgs::msg::VehicleOdometry odom{};
    odom.timestamp = px4_now_us;
    odom.timestamp_sample = px4_now_us;

    odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

    odom.position[0] = static_cast<float>(ned_p.x());
    odom.position[1] = static_cast<float>(ned_p.y());
    odom.position[2] = static_cast<float>(ned_p.z());
    
    odom.q[0] = static_cast<float>(q_frd_ned.w());
    odom.q[1] = static_cast<float>(q_frd_ned.x());
    odom.q[2] = static_cast<float>(q_frd_ned.y());
    odom.q[3] = static_cast<float>(q_frd_ned.z());

    odom.velocity[0] = NAN;
    odom.velocity[1] = NAN;
    odom.velocity[2] = NAN;

    /* GT Very low uncertainty */
    odom.position_variance[0] = 1e-4f;
    odom.position_variance[1] = 1e-4f;
    odom.position_variance[2] = 1e-4f;

    odom.orientation_variance[0] = 1e-4f;
    odom.orientation_variance[1] = 1e-4f;
    odom.orientation_variance[2] = 1e-4f;
    odom.orientation_variance[3] = 1e-4f;

    odom.quality = 100;
    odom_gt_pub_->publish(odom);
}
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gz_px4::PxOdomToTf>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}