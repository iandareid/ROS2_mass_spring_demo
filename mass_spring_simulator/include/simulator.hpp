#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "mass_spring_msgs/msg/state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace simulator
{

class Simulator : public rclcpp::Node
{
public:
    Simulator();

private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr control_sub_;
    rclcpp::Publisher<mass_spring_msgs::msg::State>::SharedPtr state_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_spring_pub_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    void controlCallback(std_msgs::msg::Float32 msg);
    void publishState();
    void rk4_step();
    void f(double (&tmp_state)[2], double (&return_vector)[2]);

    double control_cmd_;
    double state_[2];
    visualization_msgs::msg::Marker block_;
    visualization_msgs::msg::Marker spring_;
    std::vector<geometry_msgs::msg::Point> spring_points_;
    geometry_msgs::msg::Point p2_;

    double m = 2.0;
    double k = 2.5;
    double b = 0.1;
    double Ts = 0.01;
};

}

#endif