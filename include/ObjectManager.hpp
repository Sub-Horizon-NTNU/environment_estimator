#pragma once

#include "Object.hpp"
#include <USVStates.hpp>
#include <eigen3/Eigen/Dense>
#include <object_msgs/msg/object.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class ObjectManager {
public:
  ObjectManager(rclcpp::Node::SharedPtr node,
                std::shared_ptr<USVStates> usv_states)
      : node_(node), usv_states_(usv_states) {

    object_subscriber_ = node_->create_subscription<object_msgs::msg::Object>(
        "selene/object_detector/object", 10,
        std::bind(&ObjectManager::handle_detected_object, this,
                  std::placeholders::_1));

    update_objects_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(250),
        std::bind(&ObjectManager::update_predictions, this));
  }

  std::vector<Object> get_objects() const { return objects_; }

private:
  // Add in separate class/utility namespace?
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

  void handle_detected_object(
      const object_msgs::msg::Object::SharedPtr detected_object) {

    // transform object to world.
    object_msgs::msg::Object::SharedPtr object_world =
        transform_object(detected_object);

    // Get object position
    if (objects_.empty()) {
      add_object(object_world);
    } else {
      bool object_exists{};
      unsigned int object_index{};

      for (const auto &object : objects_) {
		

        if (std::hypot(object_world->position_x - object.get()->position_x,
                       object_world->position_y - object.get()->position_y) <= radius_) {

				
          update_object(object_world, object_index);
          object_exists = true;
          break;
        } 
        object_index++;
      }
      if (!object_exists) {
        add_object(object_world);
      }
    }
  }

  void add_object(const object_msgs::msg::Object::SharedPtr object) {
    RCLCPP_INFO(node_->get_logger(),"Added object: %s | position : [%.2f, %.2f]", object.get()->color.c_str(), object.get()->position_x, object.get()->position_y);

    Object new_object(object);
    // Todo figure out more actions if this becomes a issue??
    if (objects_.size() > 100) {
      RCLCPP_WARN(node_->get_logger(),"ObjectManager Buffer exceeded reasonable size, not adding more objects");
      return;
    }
    objects_.push_back(new_object);
  }

  void update_object(const object_msgs::msg::Object::SharedPtr detected_object,
                     unsigned int index) {
    // Object to be updated
    Object &object = objects_[index];
    object.update(detected_object);
    RCLCPP_INFO(node_->get_logger(),"Updated object: %s | position : [%.2f, %.2f]",object.get()->color.c_str(), object.get()->position_x, object.get()->position_y);
  }
  void remove_duplicates(){
    for(unsigned int i = 0; i< objects_.size(); i++){
      for(unsigned int j = i+1; j< objects_.size(); j++){
        double dist = std::hypot(objects_[i].get()->position_x-objects_[j].get()->position_x, objects_[i].get()->position_y-objects_[j].get()->position_y);
        if(dist <=radius_){
            objects_.erase(objects_.begin()+j);
            j--;
        }
      }
    }
  }

  void update_predictions() {
    //for (auto &object : objects_) {
    //  object.update();
    //}
    remove_duplicates();
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<USVStates> usv_states_;
  std::vector<Object> objects_;
  std::vector<Object> old_objects_;

  static constexpr double radius_ = 1.0;
  rclcpp::Subscription<object_msgs::msg::Object>::SharedPtr object_subscriber_;
  rclcpp::TimerBase::SharedPtr update_objects_timer_;
};