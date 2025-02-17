//
// Created by ken on 25-2-16.
//

#include "kalman_filter/simulator.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Simulator>("Kalman_Test");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
