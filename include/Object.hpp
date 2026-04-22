
#pragma once
#include "KalmanFilter.hpp"
#include <memory>
#include <object_msgs/msg/object.hpp>
#include <chrono>


class Object {
    public:
    explicit Object(const object_msgs::msg::Object &object);
    
    virtual void update(const object_msgs::msg::Object &object) = 0;
    
    virtual object_msgs::msg::Object get_predicted_position() = 0;
    
    object_msgs::msg::Object get() const;
    
    double get_time_since_updated();
    
    protected:
    object_msgs::msg::Object object_;
    std::chrono::steady_clock::time_point prev_time_;
    
};