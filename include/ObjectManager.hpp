#pragma once

#include "StaticObject.hpp"
#include "DynamicObject.hpp"
#include <eigen3/Eigen/Dense>
#include <object_msgs/msg/object.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "ObjectUtilities.hpp"
#include "USVTransformHandler.hpp"

#include <object_msgs/msg/boat.hpp>
#include <object_msgs/msg/boats.hpp>

#include <object_msgs/msg/buoy.hpp>
#include <object_msgs/msg/buoys.hpp>

class ObjectManager {
public:
  ObjectManager(rclcpp::Node::SharedPtr node, std::shared_ptr<USVTransformHandler> usv_transform_handler);

  std::vector<std::shared_ptr<Object>> get_objects() const;

private:

  void handle_detected_object(const object_msgs::msg::Object detected_object);

  void add_object(const object_msgs::msg::Object &object);

  void update_object(const object_msgs::msg::Object detected_object, unsigned int index);

  void remove_elements(std::vector<unsigned int> elements);

  void remove_duplicates();
  
  void publish_objects();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<USVTransformHandler> usv_transform_handler_;
  std::unique_ptr<ObjectUtilities> object_utilities_;
  std::vector<std::shared_ptr<Object>> objects_;
  bool simulator_mode_;

  static constexpr double radius_ = 1.0;
  rclcpp::Subscription<object_msgs::msg::Object>::SharedPtr object_subscriber_;
  rclcpp::TimerBase::SharedPtr update_objects_timer_;
  rclcpp::Publisher<object_msgs::msg::Buoys>::SharedPtr buoy_array_publisher_;
  rclcpp::Publisher<object_msgs::msg::Boats>::SharedPtr boat_array_publisher_;
  rclcpp::TimerBase::SharedPtr object_pub_;
};