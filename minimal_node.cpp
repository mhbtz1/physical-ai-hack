#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("minimal_node");
    RCLCPP_INFO(node->get_logger(), "Test Node Running!");
    rclcpp::spin(node);
    rclcpp::shutdown();
}