#ifndef ROS_CTRL_
#define ROS_CTRL_

#include <chrono>
#include <deque>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

struct Waypoint {
    Eigen::Vector3f p;
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
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_sub_;

    int offboard_setpoint_counter_;
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint(const Eigen::Vector3f& pos_enu, float yaw_enu);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void update_reference(float dt);

    void pos_enu_to_ned(const Eigen::Vector3f& enu, Eigen::Vector3f& ned);
    float yaw_enu_to_ned(float yaw_enu);
    float quat_to_float(const geometry_msgs::msg::Quaternion& q);
    float wrap_pi(float a);

    std::deque<Waypoint> plan_;
    rclcpp::Time last_plan_time_;
    bool have_plan_{false};

    Eigen::Vector3f p_ref_{0.0f, 0.0f, 0.5f};
    Eigen::Vector3f v_ref_{0.0f, 0.0f, 0.0f};
    float yaw_ref_{0.0f};

    float v_max_{2.0f}; // max velocity
    float a_max_{2.0f}; // max acceleration
    float lookahead_{1.5f};
    float pos_tol_{0.4f};
    float yaw_tol_{5.0f}; // degree
    float yaw_rate_max_{0.2f}; // rad/s
    float stale_timeout_{0.5f};
};


#endif