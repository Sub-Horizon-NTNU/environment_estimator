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
  ObjectManager(rclcpp::Node::SharedPtr node,
                std::shared_ptr<USVStates> usv_states)
      : node_(node), 
      usv_states_(usv_states),
      object_utilities_(std::make_unique<ObjectUtilities>(node,usv_states))
       {

    object_subscriber_ = node_->create_subscription<object_msgs::msg::Object>(
        "selene/object_detector/object", 10,
        std::bind(&ObjectManager::handle_detected_object, this,
                  std::placeholders::_1));
  }

  std::vector<Object> get_objects() const { return objects_; }

private:
  // Add in separate class/utility namespace?

  void handle_detected_object(
      const object_msgs::msg::Object::SharedPtr detected_object) {
    // transform object to world.
    object_msgs::msg::Object::SharedPtr object_world = object_utilities_->transform_object(detected_object);
    
    if (objects_.empty()) {
      add_object(object_world);
      return;
    }

    bool object_exists{};
    //Distance checks relative from USV
    
    unsigned int object_index{};

    std::vector<unsigned int> elements_to_remove{};


    //Check if object is in the radius of another object
    for (auto &object : objects_) {

        //Objects previously captured position
        double x_dist = object_world->position_x - object.get()->position_x;
        double y_dist = object_world->position_y - object.get()->position_y;

        if (std::hypot(x_dist, y_dist) <= radius_) {				
            object_exists = true;
            break;
        }

        //check stored objects predicted position
        object_msgs::msg::Object pred_object = object.get_predicted_position();
        double x_dist_pred = object_world->position_x - pred_object.position_x;
        double y_dist_pred = object_world->position_y - pred_object.position_y;
        if (std::hypot(x_dist_pred, y_dist_pred) <= radius_*2) {				
            object_exists = true;
            break;
        }

        //Check if object should be in sight and if object has been detected
        if(object_utilities_->should_be_visible(object.get()) && object.get_time_since_updated() > 1.5){
            elements_to_remove.push_back(object_index);
        }
        object_index++;
    }
    
    if(object_exists){
        update_object(object_world, object_index);
    }

    if (!object_exists) {
        add_object(object_world);
    }  
    remove_elements(elements_to_remove);

    remove_duplicates();
  }

  void add_object(const object_msgs::msg::Object::SharedPtr object) {
    RCLCPP_INFO(node_->get_logger(),"Added object: %s | pos : [%.2f, %.2f] | vel : [%.3f, %.3f]", object.get()->color.c_str(), object.get()->position_x, object.get()->position_y, object.get()->velocity_x,object.get()->velocity_y);

    // Temp, should be removed when logic/detections are better
    if (objects_.size() > 100) {
        RCLCPP_WARN(node_->get_logger(),"ObjectManager buffer exceeded reasonable size, not adding more objects");
        return;
    }
    //
    Object new_object(object);
    objects_.push_back(new_object);
  }

  void update_object(const object_msgs::msg::Object::SharedPtr detected_object, unsigned int index) {
    // Object to be updated
    Object &object = objects_[index];
    object.update(detected_object);

    //RCLCPP_INFO(node_->get_logger(),"Updated object: %s | pos : [%.2f, %.2f] | vel : [%.4f]",object.get()->color.c_str(), object.get()->position_x, object.get()->position_y, std::hypot(object.get()->velocity_x,object.get()->velocity_y));
  }

  void remove_elements(std::vector<unsigned int> elements){
    for(const auto &element : elements){
        objects_.erase(objects_.begin()+element);
    }
  }

  void remove_duplicates(){
    for(unsigned int i = 0; i< objects_.size(); i++){
      for(unsigned int j = i+1; j< objects_.size(); j++){
        double dist = std::hypot(objects_[i].get()->position_x-objects_[j].get()->position_x, objects_[i].get()->position_y-objects_[j].get()->position_y);
        if(dist <= radius_){
            objects_.erase(objects_.begin()+j);
            j--;
        }
      }
    }
  }


  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<USVStates> usv_states_;
  std::unique_ptr<ObjectUtilities> object_utilities_;
  std::vector<Object> objects_;
  std::vector<Object> old_objects_;

  static constexpr double radius_ = 1.0;
  rclcpp::Subscription<object_msgs::msg::Object>::SharedPtr object_subscriber_;
  rclcpp::TimerBase::SharedPtr update_objects_timer_;
};