#include "Object.hpp"



    Object::Object(const object_msgs::msg::Object::SharedPtr &object)
    : 
    object_(object), 
    prev_time_(std::chrono::steady_clock::now()) {}


    double Object::get_time_since_updated(){
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now-prev_time_).count();
        return dt;
    }

    object_msgs::msg::Object::SharedPtr Object::get() const {
        return object_;
    }
    
