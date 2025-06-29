//
// Created by ken on 25-2-16.
//

#include "curve_fitter/simulator.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Simulator>("Curve_Fitter");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
