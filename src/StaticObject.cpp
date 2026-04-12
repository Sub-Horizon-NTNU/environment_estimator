#include "StaticObject.hpp"

StaticObject::StaticObject(const object_msgs::msg::Object::SharedPtr &object) : 
    Object(object){

        Eigen::Matrix2d A;
        A <<
        1.0, 0.0,
        0.0, 1.0;

        Eigen::Matrix2d H;
        H <<
        1.0, 0.0,
        0.0, 1.0;

        Eigen::Vector2d x;
        x << object->position_x, object->position_y;

        Eigen::Matrix2d P = Eigen::Matrix2d::Identity();

        kalman_filter_ = std::make_shared<KalmanFilter>(x,A,H,P);

        Eigen::Matrix2d Q;
        Q <<
        0.2, 0.0,
        0.0, 0.2;
        kalman_filter_->set_process_noise_cov(Q);
    }   

void StaticObject::update(const object_msgs::msg::Object::SharedPtr &object) {

    Eigen::Vector2d z;
    Eigen::Matrix2d R;
    
    z << object->position_x, object->position_y;
    R << 
    0.1, 0.0,
    0.0, 0.1;
    
    kalman_filter_->set_measurement(z);
    kalman_filter_->set_measurement_noise_cov(R);
    kalman_filter_->update();
    Eigen::Vector2d estimates = kalman_filter_->get_estimates();

    object_->color = object->color;
    object_->type = object->type;
    object_->position_x = estimates(0);
    object_->position_y = estimates(1);
}

object_msgs::msg::Object StaticObject::get_predicted_position(){
    object_msgs::msg::Object static_object;
    static_object.position_x = object_->position_x;
    static_object.position_y = object_->position_y;
    static_object.type = "static";
    static_object.color = object_->color;

    return static_object;
}

