#pragma once

#include <rclcpp/rclcpp.hpp>
#include "ObjectManager.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

class EnvironmentVisualizer{
    public:
    
    EnvironmentVisualizer(rclcpp::Node::SharedPtr node,const std::shared_ptr<USVStates> usv_states, const std::shared_ptr<ObjectManager> object_manager);

    void publish_markers();
    void publish_usv_marker();

    private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<USVStates> usv_states_;
    std::shared_ptr<ObjectManager> object_manager_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr usv_marker_publisher_;

    rclcpp::TimerBase::SharedPtr marker_timer_;
    rclcpp::TimerBase::SharedPtr usv_timer_;


};