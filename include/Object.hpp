
#pragma once
#include "KalmanFilter.hpp"
#include "USVStates.hpp"
#include <memory>
#include <object_msgs/msg/object.hpp>
#include <chrono>


class Object {
    public:

    Object(const object_msgs::msg::Object::SharedPtr &object)
    : object_(object), prev_time_(std::chrono::steady_clock::now()){

        Eigen::MatrixXd A;
        A.setIdentity(4,4);

        Eigen::MatrixXd H(4,4);
        H << 
        1.0,0.0,0.0,0.0, 
        0.0,1.0,0.0,0.0, 
        0.0,0.0,0.0,0.0, 
        0.0,0.0,0.0,0.0; 

        Eigen::MatrixXd P;
        P.setIdentity(4,4);
        Eigen::VectorXd x(4);
        x << object->position_x,object->position_y,object->velocity_x,object->velocity_y;
        
        KF_ = std::make_shared<KalmanFilter>(x,A,H,P);
      }

    void update(const object_msgs::msg::Object::SharedPtr &object){

        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

        double dt = std::chrono::duration<double>(now-prev_time_).count();

        Eigen::VectorXd z(4);
        z << object->position_x, object->position_y, object->velocity_x, object->velocity_y;
        KF_->set_measurement(z);

        Eigen::MatrixXd A(4,4);
        A << 
        1.0, 0.0,  dt, 0.0, //x 
        0.0, 1.0, 0.0,  dt, //y
        0.0, 0.0, 1.0, 0.0, //vx
        0.0, 0.0, 0.0, 1.0; //vy

        //TODO achieve better values for process and
        //measurements cov matrix by using the variance of the heading 
        //and distance to the object.

        Eigen::MatrixXd R(4, 4);
        R <<
        0.1, 0.0, 0.0, 0.0, 
        0.0, 0.1, 0.0, 0.0,
        0.0, 0.0, 0.2, 0.0,
        0.0, 0.0, 0.0, 0.2;
        KF_->set_measurement_noise_cov(R);

        Eigen::MatrixXd Q(4, 4);
        Q <<
        0.3, 0.0, 0.0, 0.0,
        0.0, 0.3, 0.0, 0.0,
        0.0, 0.0, 0.2, 0.0,
        0.0, 0.0, 0.0, 0.2;
        KF_->set_process_noise_cov(Q);

        KF_->update();

        object_->position_x = KF_->get_estimates()(0);
        object_->position_y = KF_->get_estimates()(1);
        object_->velocity_x = KF_->get_estimates()(2);
        object_->velocity_y = KF_->get_estimates()(3);
        prev_time_ = now;
    }

    void update(){
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now-prev_time_).count();

        Eigen::MatrixXd A(4,4);
        A << 
        1.0, 0.0,  dt, 0.0, //x 
        0.0, 1.0, 0.0,  dt, //y
        0.0, 0.0, 1.0, 0.0, //vx
        0.0, 0.0, 0.0, 1.0; //vy

        KF_->predict();
    }

    object_msgs::msg::Object::SharedPtr get() const {
        return object_;
    }
    
    double get_radius() const {
        return radius_;
    }

    object_msgs::msg::Object::SharedPtr object_;
    double radius_;

    std::chrono::steady_clock::time_point prev_time_;
    std::shared_ptr<KalmanFilter> KF_;

};