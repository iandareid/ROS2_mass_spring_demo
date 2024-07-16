#include "simulator.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<simulator::Simulator>());

    return 0;
}
