#pragma once
#include <memory>
#include <object_msgs/msg/detail/object__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include "USVStates.hpp"
#include <object_msgs/msg/object.hpp>
#include <eigen3/Eigen/Dense>


//Class to manage objects based on the USVs position and orientation
class ObjectUtilities{
    public:
    ObjectUtilities(rclcpp::Node::SharedPtr node,const std::shared_ptr<USVStates> usv_states);

     object_msgs::msg::Object::SharedPtr transform_object(const object_msgs::msg::Object::SharedPtr object_sensor_frame);

    bool should_be_visible(const object_msgs::msg::Object::SharedPtr &object);

    double get_accuracy(const object_msgs::msg::Object::SharedPtr &object);

    private: 
    bool is_inside_radius(const object_msgs::msg::Object::SharedPtr &object);

    bool is_inside_fov(const object_msgs::msg::Object::SharedPtr &object);

    private: 
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<USVStates> usv_states_;
        float field_of_view_;
        float max_radius_;
        float min_radius_;
};