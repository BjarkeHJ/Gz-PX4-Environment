#ifndef ROS_CTRL_
#define ROS_CTRL_

#include <chrono>
#include <queue>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

struct Pose {
    Eigen::Vector3f pos;
    float yaw;
};

class RosControl : public rclcpp::Node 
{
public:
    RosControl();

    void arm();
    void disarm();

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;

    int offboard_setpoint_counter_;
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint(const Eigen::Vector3f& pos_enu, float yaw_enu);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    void pos_enu_to_ned(const Eigen::Vector3f& enu, Eigen::Vector3f& ned);
    float yaw_enu_to_ned(float yaw_enu);
    float quat_to_float(const geometry_msgs::msg::Quaternion& q);

    float temp_yaw = 0.0f;
    const Eigen::Vector3f temp_pos{0.0, 0.0, 5.0};

    std::queue<Eigen::Vector3f> pos_queue;
    std::queue<float> yaw_queue;

    std::queue<Pose> next_waypoint_interp;
};


#endif