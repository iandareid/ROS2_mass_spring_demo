#include "rclcpp/rclcpp.hpp"
#include "mass_spring_msgs/msg/state.hpp"
#include "std_msgs/msg/float32.hpp"
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
    rclcpp::TimerBase::SharedPtr update_timer_;

    void controlCallback(std_msgs::msg::Float32 msg);
    void publishState();
    void rk4_step();
    void f(double (&tmp_state)[2], double (&return_vector)[2]);

    double control_cmd_;
    double state_[2];

    double m = 2.0;
    double k = 2.5;
    double b = 0.1;
    double Ts = 0.01;
};

Simulator::Simulator() : Node("simulator")
{
    // set up publishers and subscribers
    control_sub_ = this->create_subscription<std_msgs::msg::Float32>("control", 10,
        std::bind(&Simulator::controlCallback, this, _1));
    state_pub_ = this->create_publisher<mass_spring_msgs::msg::State>("state", 10);

    // Set up timer
    update_timer_ = this->create_wall_timer(10ms, std::bind(&Simulator::publishState, this));

    // Initialize variables
    control_cmd_ = 0.0;
    for (int i=0; i<2; i++) {
        state_[i] = 0.0;
    }
}

void Simulator::controlCallback(std_msgs::msg::Float32 msg) {
    control_cmd_ = msg.data; 
}

void Simulator::publishState() {
    // Insert integration here
    rk4_step();

    mass_spring_msgs::msg::State msg;
    msg.position = state_[0];
    msg.velocity = state_[1];
    state_pub_->publish(msg);
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

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<simulator::Simulator>());

    return 0;
}