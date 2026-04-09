#pragma once

#include "Object.hpp"
#include <USVStates.hpp>
#include <eigen3/Eigen/Dense>
#include <object_msgs/msg/object.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "ObjectUtilities.hpp"

class ObjectManager {
public:
  ObjectManager(rclcpp::Node::SharedPtr node, std::shared_ptr<USVStates> usv_states);

  std::vector<Object> get_objects() const;

private:

  void handle_detected_object(const object_msgs::msg::Object::SharedPtr detected_object);

  void add_object(const object_msgs::msg::Object::SharedPtr object);

  void update_object(const object_msgs::msg::Object::SharedPtr detected_object, unsigned int index);

  void remove_elements(std::vector<unsigned int> elements);

  void remove_duplicates();


  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<USVStates> usv_states_;
  std::unique_ptr<ObjectUtilities> object_utilities_;
  std::vector<Object> objects_;
  std::vector<Object> old_objects_;

  static constexpr double radius_ = 1.0;
  rclcpp::Subscription<object_msgs::msg::Object>::SharedPtr object_subscriber_;
  rclcpp::TimerBase::SharedPtr update_objects_timer_;
};