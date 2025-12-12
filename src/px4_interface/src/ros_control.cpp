#include "ros_control.hpp"

RosControl::RosControl() : Node("ros_control") {
    offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    waypoint_sub_ = this->create_subscription<nav_msgs::msg::Path>("/waypoints_path", 5, std::bind(&RosControl::path_callback, this, std::placeholders::_1));

    offboard_setpoint_counter_ = 0;

    auto timer_callback = [this]() -> void {
        if (offboard_setpoint_counter_ == 10) {
            // After 10 setpoint change to offboard control mode
            this -> publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            // Arm the vehicle
            this -> arm();
        }

        publish_offboard_control_mode();
        const float dt = 0.05f; // 50 hz
        update_reference(dt);
        publish_trajectory_setpoint(p_ref_, yaw_ref_);

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

void RosControl::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    last_plan_time_ = this->get_clock()->now();

    if (msg->poses.size() < 1) {
        RCLCPP_WARN(this->get_logger(), "Received Empty Path - Nothing to do!");
        have_plan_ = false;
        plan_.clear();
        return;
    }
    plan_.clear();
    plan_.push_back(Waypoint{p_ref_, yaw_ref_});

    // Incoming waypoints
    for (const auto& ps : msg->poses) {
        Eigen::Vector3f p(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
        float yaw = quat_to_float(ps.pose.orientation);
        plan_.push_back({p, yaw});
    }

    have_plan_ = (plan_.size() >= 2 );
}

void RosControl::update_reference(float dt) {
    if (!have_plan_ || (this->get_clock()->now() - last_plan_time_).seconds() > stale_timeout_) {
        // brake to stop smoothly
        Eigen::Vector3f dv = -v_ref_;
        float dv_norm = dv.norm();
        float dv_max = a_max_ * dt;
        if (dv_norm > dv_max) {
            dv *= (dv_max / dv_norm);
        }
        v_ref_ += dv;
        p_ref_ += v_ref_ * dt;
        return;
    }

    while (plan_.size() >= 2) {
        Eigen::Vector3f to_wp = plan_[1].p - p_ref_;
        if (to_wp.norm() < pos_tol_) {
            plan_.pop_front(); // advance waypoint
        }
        else {
            break;
        }
    }

    if (plan_.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "[Trajectory Generator] Less than 2 waypoints given... Cannot generate trajectory!");
        have_plan_ = false;
        return;
    }

    Eigen::Vector3f target = plan_[1].p;
    Eigen::Vector3f d = target - p_ref_;
    float dist = d.norm();
    Eigen::Vector3f dir = Eigen::Vector3f::Zero();
    if (dist > 1e-6f) dir = d / dist;
    
    float step = std::min(lookahead_, dist);
    Eigen::Vector3f carrot = p_ref_ + dir * step;
    
    Eigen::Vector3f v_des = (carrot - p_ref_);
    float v_des_norm = v_des.norm();
    if (v_des_norm > 1e-6f) {
        v_des = v_des / v_des_norm * std::min(v_max_, v_des_norm / dt); // velocity-ish
    }
    else {
        v_des.setZero();
    }

    // limit acceleration
    Eigen::Vector3f dv = v_des - v_ref_;
    float dv_norm = dv.norm();
    float dv_max = a_max_ * dt;
    if (dv_norm > dv_max) {
        dv *= (dv_max / dv_norm);
    }
    v_ref_ += dv;
    p_ref_ += v_ref_ * dt; // integrate pos;
    
    // limit yaw rate
    float yaw_des = yaw_ref_;
    if (v_ref_.head<2>().norm() > 0.2f) {
        yaw_des = std::atan2(v_ref_.y(), v_ref_.x());
    }

    float dy = wrap_pi(yaw_des - yaw_ref_);
    float max_dy = yaw_rate_max_ * dt;
    dy = std::clamp(dy, -max_dy, +max_dy);
    yaw_ref_ = wrap_pi(yaw_ref_ + dy);
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

float RosControl::wrap_pi(float a) {
    while (a > M_PI) a -= 2.f * M_PI;
    while (a < -M_1_PI) a += 2.f * M_PI;
    return a;
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RosControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}