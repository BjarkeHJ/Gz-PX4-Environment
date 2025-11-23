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
    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = "world";        // parent
    tf_msg.child_frame_id  = "base_link";  // child

    // tf_msg.transform.translation.x = msg->position[0];
    // tf_msg.transform.translation.y = msg->position[1];
    // tf_msg.transform.translation.z = msg->position[2];

    tf_msg.transform.translation.x = msg->position[1];
    tf_msg.transform.translation.y = msg->position[0];
    tf_msg.transform.translation.z = -msg->position[2];

    tf_msg.transform.rotation.x = msg->q[0];
    tf_msg.transform.rotation.y = msg->q[1];
    tf_msg.transform.rotation.z = msg->q[2];
    tf_msg.transform.rotation.w = msg->q[3];

    tf_broadcaster_->sendTransform(tf_msg);

    std::cout << "PUBLISHED TRANSFORM!" << std::endl;
}

}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gz_px4::PxOdomToTf>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}