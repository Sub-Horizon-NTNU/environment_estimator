#include "DynamicObject.hpp"
#include <memory>

DynamicObject::DynamicObject(const object_msgs::msg::Object::SharedPtr &object) : 
    Object(object),
    position_x_(object->position_x),
    position_y_(object->position_y){

        Eigen::MatrixXd A = Eigen::MatrixXd::Identity(6,6);
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, 6);

        H <<
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0;

        Eigen::VectorXd x(6);
        x << object->position_x, object->position_y, 0.0, 0.0, 0.0, 0.0; // [px,py,vx,vy,ax,ay]

        Eigen::MatrixXd P = Eigen::MatrixXd::Identity(6,6);

        kalman_filter_ = std::make_shared<KalmanFilter>(x,A,H,P);

        Eigen::MatrixXd Q(6,6);
        Q <<
        0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.1;

        kalman_filter_->set_process_noise_cov(Q);
    }   

void DynamicObject::update(const object_msgs::msg::Object::SharedPtr &object) {

    Eigen::VectorXd z(4);
    Eigen::MatrixXd R(4,4);
    Eigen::MatrixXd A(6,6);
    
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

    double dt = std::chrono::duration<double>(now-prev_time_).count();
    double velocity_x = (object_->position_x-position_x_)/dt;
    double velocity_y = (object_->position_y-position_y_)/dt;
    
    z << object->position_x, object->position_y, velocity_x, velocity_y;

    A <<
    1.0, 0.0, dt,  0.0,  dt*dt*0.5, 0.0, //px
    0.0, 1.0, 0.0, dt,  0.0, dt*dt*0.5, //py
    0.0, 0.0, 1.0, 0.0, dt, 0.0, //vx
    0.0, 0.0, 0.0, 1.0, 0.0, dt, //vy
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, //ax
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0; //ay

    R << 
    0.1, 0.0, 0.0, 0.0,
    0.0, 0.1, 0.0, 0.0,
    0.0, 0.0, 0.1, 0.0,
    0.0, 0.0, 0.0, 0.1;
    
    kalman_filter_->set_model(A);
    kalman_filter_->set_measurement(z);
    kalman_filter_->set_measurement_noise_cov(R);
    kalman_filter_->update();
    Eigen::VectorXd estimates = kalman_filter_->get_estimates();

    object_->color = object->color;
    object_->type = object->type;
    object_->position_x = estimates(0);
    object_->position_y = estimates(1);
    object_->velocity_x = estimates(2);
    object_->velocity_y = estimates(3);
    object_->acceleration_x = estimates(4);
    object_->acceleration_y = estimates(5);
   
}

object_msgs::msg::Object DynamicObject::get_predicted_position(){
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now-prev_time_).count();
    //Use constant acceleration model to predict the objects position
    Eigen::MatrixXd A(6,6);
    A <<
    1.0, 0.0, dt,  0.0,  0.0, 0.0, //px
    0.0, 1.0, 0.0, dt,  0.0, 0.0, //py
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, //vx
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, //vy
    0.0, 0.0, 0.0, 0.0, dt*dt*0.5, 0.0, //ax
    0.0, 0.0, 0.0, 0.0, 0.0, dt*dt*0.5; //ay

    Eigen::VectorXd predicted = kalman_filter_->get_predicted_state(A);
    object_msgs::msg::Object predicted_object;
    predicted_object.position_x = predicted(0);
    predicted_object.position_y = predicted(1);
    predicted_object.velocity_x = predicted(2);
    predicted_object.velocity_y = predicted(3);
    predicted_object.acceleration_x = predicted(4);
    predicted_object.acceleration_y = predicted(5);
    
    return predicted_object;
    }