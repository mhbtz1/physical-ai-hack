#include <sensor_msgs/msg/joint_state.hpp>

// This will subscribe to the /join_states topic, with a message type of sensor_msgs/JointState

void joint_cb(const sensor_msgs::msg::JointState::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("JointStateSubscriber"), "Received joint state: %s", msg->name.c_str());

    for (size_t i = 0; i < msg->name.size(); i++) {
        RCLCPP_INFO(get_logger(0, "%s: pos=$.3f vel=$.3f effort=%.3f", msg->name[i].c_str(), msg->position[i], msg->velocity[i], msg->effort[i]));
    }

}