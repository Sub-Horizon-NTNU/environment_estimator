#include "ObjectManager.hpp"
#include <chrono>
#include <rclcpp/qos.hpp>

  ObjectManager::ObjectManager(rclcpp::Node::SharedPtr node, std::shared_ptr<USVTransformHandler> usv_transform_handler)
      : node_(node), 
        usv_transform_handler_(usv_transform_handler),
        object_utilities_(std::make_unique<ObjectUtilities>(node,usv_transform_handler))
       {

        object_subscriber_ = node_->create_subscription<object_msgs::msg::Object>(
            "selene/object_detector/object", 10,
            std::bind(&ObjectManager::handle_detected_object, this,std::placeholders::_1));

        buoy_array_publisher_ = node_->create_publisher<object_msgs::msg::Buoys>(
            "selene/environment_estimator/buoys",
            10
        );

        boat_array_publisher_ = node_->create_publisher<object_msgs::msg::Boats>(
            "selene/environment_estimator/boats",
            10
        );

        object_pub_ = node_->create_wall_timer(
            std::chrono::milliseconds(600),
            std::bind(&ObjectManager::publish_objects, this));

        node_->declare_parameter("simulator_mode",false);
        simulator_mode_ = node_->get_parameter("simulator_mode").as_bool();
  }

  std::vector<std::shared_ptr<Object>> ObjectManager::get_objects() const { return objects_; }

  void ObjectManager::handle_detected_object(const object_msgs::msg::Object detected_object) {

    object_msgs::msg::Object object_world;

    if(simulator_mode_){
        object_world = object_utilities_->transform_object(detected_object); // transform object to world.
    } else {
        object_world = usv_transform_handler_->camera_to_world(detected_object);
    }
    
    if (objects_.empty()) { // if empty then the first object can just be added, without any checks
      add_object(object_world);
      return;
    }

    bool object_exists{}; 
    unsigned int object_index{};
    std::vector<unsigned int> elements_to_remove{};

    //Check if object is in the radius of another object
    for (auto &object : objects_) {
        //Objects previously captured position
        double x_dist = object_world.position_x - object->get().position_x;
        double y_dist = object_world.position_y - object->get().position_y;

        if (std::hypot(x_dist, y_dist) <= radius_) {				
            object_exists = true;
            break;
        }
        //check stored objects predicted position
        if(object->get().type == "dynamic"){
            object_msgs::msg::Object pred_object = object->get_predicted_position();
            double x_dist_pred = object_world.position_x - pred_object.position_x;
            double y_dist_pred = object_world.position_y - pred_object.position_y;
            if (std::hypot(x_dist_pred, y_dist_pred) <= radius_) {				
                object_exists = true;
                break;
            }
        }
        //Check if object should be in sight and if object has been detected
        if(object_utilities_->should_be_visible(object->get()) && object->get_time_since_updated() > 1.5){
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

    remove_elements(elements_to_remove);  //Removes marked elements in the object_vector

    remove_duplicates();
  }

  void ObjectManager::add_object(const object_msgs::msg::Object &object) {
        //RCLCPP_INFO(node_->get_logger(),"Added object: %s | pos : [%.2f, %.2f] | vel : [%.3f, %.3f]", object.type.c_str(), object.position_x, object.position_y, object.velocity_x,object.velocity_y);
        if(object.type == "static"){
            objects_.push_back(std::make_shared<StaticObject>(object));
        }
        else if(object.type == "dynamic"){
            objects_.push_back(std::make_shared<DynamicObject>(object));
        } 
    }

  void ObjectManager::update_object(const object_msgs::msg::Object detected_object, unsigned int index) {
    // Object to be updated
    std::shared_ptr<Object> &object = objects_[index];
    object->update(detected_object);

    //RCLCPP_INFO(node_->get_logger(),"Updated object: %s | pos : [%.2f, %.2f] | vel : [%.4f]",object.get()->color.c_str(), object.get()->position_x, object.get()->position_y, std::hypot(object.get()->velocity_x,object.get()->velocity_y));
  }

    void ObjectManager::remove_elements(std::vector<unsigned int> elements){
        for(const auto &element : elements){
            objects_.erase(objects_.begin()+element);
        }
    }

    void ObjectManager::remove_duplicates(){
        for(unsigned int i = 0; i< objects_.size(); i++){
            for(unsigned int j = i+1; j< objects_.size(); j++){
                double dist = std::hypot(objects_[i]->get().position_x-objects_[j]->get().position_x, objects_[i]->get().position_y-objects_[j]->get().position_y);
                if(dist <= radius_){
                    objects_.erase(objects_.begin()+j);
                    j--;
                }
            }
        }
    }

    void ObjectManager::publish_objects(){

        object_msgs::msg::Boats boats;
        object_msgs::msg::Buoys buoys;

        for(const auto &object: objects_){
            if(object->get().type == "dynamic"){
                object_msgs::msg::Boat boat;

                boat.pos_x = object->get().position_x;
                boat.pos_y = object->get().position_y;
                boat.velocity_x = object->get().velocity_x;
                boat.velocity_y = object->get().velocity_y;
                boat.acceleration_x = object->get().acceleration_x;
                boat.acceleration_y = object->get().acceleration_y;
                boat.id = object->get().id;
                boat.color = object->get().color;
                boat.type = "unknown";

                boats.boats.push_back(boat);
            }

            if(object->get().type == "static"){
                object_msgs::msg::Buoy buoy;
                buoy.x = object->get().position_x;
                buoy.y = object->get().position_y;
                buoy.color = object->get().color;
                buoy.id = object->get().id;
                buoys.buoys.push_back(buoy);
            }
        }

        if(boats.boats.size()>0){
            boat_array_publisher_->publish(boats);
        }
        if(buoys.buoys.size()>0){
            buoy_array_publisher_->publish(buoys);
        }

    }

