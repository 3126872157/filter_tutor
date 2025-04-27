//
// Created by ken on 25-2-16.
//

#include "kalman_filter/simulator.h"

//模拟器构造函数
Simulator::Simulator(const std::string& name) : Node(name), generator_(std::random_device{}())
{
    //初始化匀加速型卡尔曼滤波器，delta_T设置为0.01即100Hz
    kalman_filter_CA_ = std::make_unique<Kalman>(0.01);
    kalman_filter_CA_->Q_set(100);
    kalman_filter_CA_->R_set(0.01);

    this->declare_parameter("Kalman_Q", 100.0);
    this->declare_parameter("Kalman_R", 0.01);

    //动态参数设置
    params_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&Simulator::param_callback, this, std::placeholders::_1));

    //并行回调，设置为同一组别
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    //发布原始数据
    raw_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("raw", 1);
    //发送滤波后数据
    filtered_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("filtered", 1);
    //原始数据生成定时器
    simulator_timer_ = create_wall_timer(std::chrono::milliseconds(1),
                                         [this]() { simulator_callback(); },
                                         callback_group_);
    //生成过滤后数据
    kalman_timer_ = create_wall_timer(std::chrono::milliseconds(1),
                                      [this]() { kalman_callback(); }, callback_group_);
    //构造成功
    RCLCPP_INFO(this->get_logger(), "%s node initialized!", name.c_str());
}

//生成搞死噪音
double Simulator::generateNoise(double mean, double stddev)
{
    std::normal_distribution<> distribution(mean, stddev);
    return distribution(generator_);
}

//模拟器生成原始数据，模拟前端传入数据频率
void Simulator::simulator_callback()
{
    static double pos_x = 0;
    static double vec_x = 0;
    static double acc_x = 0.5;
    static double delta_t = 0.01;
    static uint16_t error_injeect_count = 0;

    // RCLCPP_INFO(this->get_logger(), "Simulator callback");

    //运动方程
    if (vec_x > 2)
    {
        acc_x = -0.5;
    }
    else if (vec_x < -2)
    {
        acc_x = 0.5;
    }
    vec_x += acc_x * delta_t;
    pos_x = pos_x + vec_x * delta_t + 0.5 * acc_x * delta_t * delta_t;

    simulate_pos_x = pos_x + generateNoise(0, 0.05);

    //故障注入
    if (error_injeect_count > 150)
    {
        error_injeect_count = 0;
        simulate_pos_x += 2;
    }
    error_injeect_count += 1;

    //数据产生结束
    sim_running_ = false;

    //发布原始数据
    geometry_msgs::msg::Vector3 raw_data;
    raw_data.x = simulate_pos_x;
    raw_data.y = vec_x;
    raw_data.z = acc_x;
    raw_pub_->publish(raw_data);
}

//生成滤波后数据，以云台的响应频率发出
void Simulator::kalman_callback()
{
    // RCLCPP_INFO(this->get_logger(), "Kalman callback");

    if (sim_running_ == false)
    {
        Eigen::Matrix<double, 1, 1> measurement;
        measurement(0) = simulate_pos_x;

        kalman_filter_CA_->predict();
        auto kalman_output = kalman_filter_CA_->update(measurement);

        geometry_msgs::msg::Vector3 filtered_data;
        filtered_data.x = kalman_output(0);
        filtered_data.y = kalman_output(1);
        filtered_data.z = kalman_output(2);

        //滤波完成
        filtered_pub_->publish(filtered_data);
        sim_running_ = true;
    }
}

rcl_interfaces::msg::SetParametersResult Simulator::param_callback(const std::vector<rclcpp::Parameter>& params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : params)
    {
        if (param.get_name() == "Kalman_Q")
        {
            RCLCPP_INFO(this->get_logger(), "Q update : %f", param.as_double());
            kalman_filter_CA_->Q_set(param.as_double());
        }
        if (param.get_name() == "Kalman_R")
        {
            RCLCPP_INFO(this->get_logger(), "R update : %f", param.as_double());
            kalman_filter_CA_->R_set(param.as_double());
        }
    }
    return result;
}
