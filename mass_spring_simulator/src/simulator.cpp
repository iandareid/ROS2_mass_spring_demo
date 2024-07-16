#include "simulator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "mass_spring_msgs/msg/state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace simulator
{

Simulator::Simulator() : Node("simulator")
{
    // set up publishers and subscribers
    control_sub_ = this->create_subscription<std_msgs::msg::Float32>("control", 10,
        std::bind(&Simulator::controlCallback, this, _1));
    state_pub_ = this->create_publisher<mass_spring_msgs::msg::State>("state", 10);
    rviz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("mass", 10);
    rviz_spring_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("spring", 10);

    // Set up timer
    update_timer_ = this->create_wall_timer(10ms, std::bind(&Simulator::publishState, this));

    // Initialize variables
    control_cmd_ = 0.0;
    for (int i=0; i<2; i++) {
        state_[i] = 0.0;
    }

    block_.header.frame_id = "map";
    block_.ns = "block";
    block_.id = 0;
    block_.type = visualization_msgs::msg::Marker::CUBE;
    block_.action = visualization_msgs::msg::Marker::ADD;
    block_.pose.position.x = state_[0] + 3.0;
    block_.pose.position.y = 0.0;
    block_.pose.position.z = 0.5;
    block_.scale.x = 1.0;
    block_.scale.y = 1.0;
    block_.scale.z = 1.0;
    block_.color.r = 1.0f;
    block_.color.g = 0.0f;
    block_.color.b = 0.0f;
    block_.color.a = 1.0;

    geometry_msgs::msg::Point p1;
    p1.x = 0.0;
    p1.y = 0.0;
    p1.z = 0.5;
    p2_.x = state_[0] + 3.0;
    p2_.y = 0.0;
    p2_.z = 0.5;
    spring_points_.push_back(p1);
    spring_points_.push_back(p2_);

    spring_.header.frame_id = "map";
    spring_.ns = "spring";
    spring_.id = 0;
    spring_.type = visualization_msgs::msg::Marker::LINE_LIST;
    spring_.action = visualization_msgs::msg::Marker::ADD;
    spring_.scale.x = 0.2;
    spring_.color.r = 0.0f;
    spring_.color.g = 1.0f;
    spring_.color.b = 0.0f;
    spring_.color.a = 1.0;
    spring_.points = spring_points_;
}

void Simulator::controlCallback(std_msgs::msg::Float32 msg) {
    control_cmd_ = msg.data; 
}

void Simulator::publishState() {
    // Insert integration here
    rk4_step();

    // Publish the state
    mass_spring_msgs::msg::State msg;
    msg.position = state_[0];
    msg.velocity = state_[1];
    state_pub_->publish(msg);

    // Publish to Rviz
    block_.pose.position.x = state_[0] + 3.0;
    p2_.x = state_[0] + 3.0;
    spring_points_[1] = p2_;
    spring_.points = spring_points_;

    rviz_pub_->publish(block_);
    rviz_spring_pub_->publish(spring_);

    // RCLCPP_INFO_STREAM(this->get_logger(), sizeof(spring_points_) / sizeof(geometry_msgs::msg::Point));
}

void Simulator::rk4_step() {
    double tmp[2];
    double F1[2];
    double F2[2];
    double F3[2];
    double F4[2];

    f(state_, F1);

    for (int i=0; i<2; ++i) {
        tmp[i] = F1[i] * Ts / 2 + state_[i];
    }
    f(tmp, F2);
    
    for (int i=0; i<2; ++i) {
        tmp[i] = F2[i] * Ts / 2 + state_[i];
    }
    f(tmp, F3);

    for (int i=0; i<2; ++i) {
        tmp[i] = F3[i] * Ts + state_[i];
    }
    f(tmp, F4);

    for (int i=0; i <2; ++i) {
        state_[i] += Ts / 6 * (F1[i] + 2*F2[i] + 2*F3[i] + F4[i]);
    }
}

void Simulator::f(double (&tmp_state)[2], double (&return_vector)[2]) {
    double x = tmp_state[0];
    double x_dot = tmp_state[1];
    double x_ddot = (-b*x_dot - k*x + control_cmd_) / m;
    
    return_vector[0] = x_dot;
    return_vector[1] = x_ddot;
}

} // namespace simulator