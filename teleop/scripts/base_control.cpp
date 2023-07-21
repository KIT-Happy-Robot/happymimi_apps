
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include "your_package/msg/car_control.hpp"  // カスタムメッセージをインクルード

class CarControlNode : public rclcpp::Node {
public:
    CarControlNode() : Node("car_control_node") {
        publisher_ = this->create_publisher<your_package::msg::CarControl>("car_control_topic", 10);

        // 制御情報の更新処理を行うタイマーを設定
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CarControlNode::updateControl, this));
    }
