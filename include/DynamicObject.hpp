
#pragma once 
#include "Object.hpp"
#include "KalmanFilter.hpp"

class DynamicObject : public Object {
    public:
    DynamicObject(const object_msgs::msg::Object &object);

    void update(const object_msgs::msg::Object &object) override;

    object_msgs::msg::Object get_predicted_position() override;

    object_msgs::msg::Object predict_states(const double &seconds,const double &dt) override;

    private:
        std::shared_ptr<KalmanFilter> kalman_filter_;
        double position_x_{};
        double position_y_{};
};