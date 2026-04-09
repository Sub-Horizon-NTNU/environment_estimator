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
    ObjectUtilities(rclcpp::Node::SharedPtr node,const std::shared_ptr<USVStates> usv_states):
    node_(node),
    usv_states_(usv_states)
    {
        node->declare_parameter<double>("field_of_view", 78.0);
        node->declare_parameter<double>("max_radius", 20.0);
        node->declare_parameter<double>("min_radius", 1.0);
    }
     object_msgs::msg::Object::SharedPtr transform_object(
      const object_msgs::msg::Object::SharedPtr object_sensor_frame) {

    double cos_h = std::cos(usv_states_->get_states().heading);
    double sin_h = std::sin(usv_states_->get_states().heading);

    Eigen::Vector2d usv_pos(usv_states_->get_states().x,
                            usv_states_->get_states().y);

    Eigen::Matrix<double, 2, 2> T_object_boat;
    T_object_boat << cos_h, -sin_h, sin_h, cos_h;

    Eigen::Vector2d rel_pos;
    rel_pos << object_sensor_frame->position_x, object_sensor_frame->position_y;

    Eigen::Vector2d world_pos = T_object_boat * rel_pos + usv_pos;

    auto object_world = std::make_shared<object_msgs::msg::Object>();

    object_world->position_x = world_pos.x();
    object_world->position_y = world_pos.y();
    object_world->color = object_sensor_frame->color;
    object_world->id = object_sensor_frame->id;
    // RCLCPP_INFO(node_->get_logger(), "USV state | x: %.2f, y: %.2f, heading:
    // %.2f | obj_sensor: [%.2f, %.2f]", usv_states_->get_states().x,
    // usv_states_->get_states().y,
    // usv_states_->get_states().heading,
    // object_sensor_frame->position_x,
    // object_sensor_frame->position_y);
    return object_world;
  }

    bool should_be_visible(const object_msgs::msg::Object::SharedPtr &object){
        return is_inside_radius(object) && is_inside_fov(object);
    }
    bool is_inside_radius(const object_msgs::msg::Object::SharedPtr &object){
        double dist = std::hypot(usv_states_->get_states().x-object->position_x,usv_states_->get_states().y-object->position_y);
        RCLCPP_INFO(node_->get_logger(),"distance: %.2f",dist);
        if(dist > 20.0 or dist <0.5){
            return false;
        }
        return true;
    }

    bool is_inside_fov(const object_msgs::msg::Object::SharedPtr &object){
        double x_diff = object->position_x- usv_states_->get_states().x;
        double y_diff = object->position_y- usv_states_->get_states().y;

        double angle_diff = usv_states_->get_states().heading-std::atan2(y_diff,x_diff);
        while (angle_diff >  M_PI){angle_diff -= 2.0 * M_PI;}
        while (angle_diff < -M_PI){angle_diff += 2.0 * M_PI;}

        RCLCPP_INFO(node_->get_logger(),"Angle: %.2f",angle_diff*180/M_PI);

        if(std::abs(angle_diff*180/M_PI) > 78.0){
            return false;
        }
        return true;
    }

    private: 
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<USVStates> usv_states_;
};