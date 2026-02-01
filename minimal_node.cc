#include <rclcpp/rclcpp.hpp>
#include <memory>

class MinimalNode: public rclcpp::Node {
    public: 
        MinimalNode(): Node("minimal_node") {
            RCLCPP_INFO(get_logger(), "Node initialized");
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}