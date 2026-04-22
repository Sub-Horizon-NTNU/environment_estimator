#pragma once
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <object_msgs/msg/object.hpp>
#include <eigen3/Eigen/Dense>
#include "USVTransformHandler.hpp"


//Class to manage objects based on the USVs position and orientation
class ObjectUtilities{
    public:
    ObjectUtilities(rclcpp::Node::SharedPtr node,const std::shared_ptr<USVTransformHandler> usv_transform_handler);

     object_msgs::msg::Object transform_object(const object_msgs::msg::Object object_sensor_frame);

    bool should_be_visible(const object_msgs::msg::Object &object);

    double get_accuracy(const object_msgs::msg::Object &object);

    private: 
    bool is_inside_radius(const object_msgs::msg::Object &object);

    bool is_inside_fov(const object_msgs::msg::Object &object);

    private: 
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<USVTransformHandler> usv_transform_handler_;
        float field_of_view_;
        float max_radius_;
        float min_radius_;
};