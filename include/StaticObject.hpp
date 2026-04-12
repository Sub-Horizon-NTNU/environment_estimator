#pragma once 
#include "Object.hpp"
#include "KalmanFilter.hpp"
#include <memory>

class StaticObject : public Object {
    public:
    StaticObject(const object_msgs::msg::Object::SharedPtr &object);

    void update(const object_msgs::msg::Object::SharedPtr &object) override;

    object_msgs::msg::Object get_predicted_position() override;
    

    private:
        std::shared_ptr<KalmanFilter> kalman_filter_;

};