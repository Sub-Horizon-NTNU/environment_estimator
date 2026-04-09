
#pragma once
#include "KalmanFilter.hpp"
#include "USVStates.hpp"
#include <memory>
#include <object_msgs/msg/object.hpp>
#include <chrono>


class Object {
    public:

    Object(const object_msgs::msg::Object::SharedPtr &object);

    //Seconds
    double get_time_since_updated();

    void update(const object_msgs::msg::Object::SharedPtr &object);

    object_msgs::msg::Object get_predicted_position();

    object_msgs::msg::Object::SharedPtr get() const;
    
    object_msgs::msg::Object::SharedPtr object_;
    std::chrono::steady_clock::time_point prev_time_;

    double position_x_; // Raw previously detected position
    double position_y_;

    std::shared_ptr<KalmanFilter> KF_;

};