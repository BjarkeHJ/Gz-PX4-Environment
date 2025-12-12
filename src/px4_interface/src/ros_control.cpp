#include "ros_control.hpp"

RosControl::RosControl() : Node("ros_control") {
    offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

    offboard_setpoint_counter_ = 0;

    auto timer_callback = [this]() -> void {
        if (offboard_setpoint_counter_ == 10) {
            // After 10 setpoint change to offboard control mode
            this -> publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            // Arm the vehicle
            this -> arm();
        }

        publish_offboard_control_mode();
        publish_trajectory_setpoint(temp_pos, temp_yaw);

        if (offboard_setpoint_counter_ < 11) {
            offboard_setpoint_counter_++;
        }
    };

    // Maybe change timer to sim-time later (but ok for now - If RTF is increased this timer rate is decreased in practice)
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), timer_callback);

    RCLCPP_INFO(this->get_logger(), "Starting Offboard Control node (RosControl)...");
}

void RosControl::arm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command sent!");
}

void RosControl::disarm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command sent!");
}

void RosControl::publish_offboard_control_mode() {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000; // sim time
    offboard_control_mode_pub_->publish(msg);
}

void RosControl::publish_trajectory_setpoint(const Eigen::Vector3f& pos_enu, float yaw_enu) {
    Eigen::Vector3f pos_ned;
    pos_enu_to_ned(pos_enu, pos_ned);
    float yaw_ned = yaw_enu_to_ned(yaw_enu);

    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {pos_ned.x(), pos_ned.y(), pos_ned.z()};
    msg.yaw = yaw_ned;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_pub_->publish(msg);
}

void RosControl::publish_vehicle_command(uint16_t command, float param1, float param2) {
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000; // sim time
    vehicle_command_pub_->publish(msg);
}

void RosControl::pos_enu_to_ned(const Eigen::Vector3f& enu, Eigen::Vector3f& ned) {
    ned.x() = enu.y(); // north
    ned.y() = enu.x(); // east
    ned.z() = -enu.z(); // down
}

float RosControl::yaw_enu_to_ned(float yaw_enu) {
    return yaw_enu + M_PI_2; // +90 deg
}

float RosControl::quat_to_float(const geometry_msgs::msg::Quaternion& q) {
    Eigen::Quaternionf quat(q.w, q.x, q.y, q.z);
    Eigen::Matrix3f R = quat.toRotationMatrix();
    float yaw = std::atan2(R(1,0), R(0,0));
    return yaw;
}



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RosControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}