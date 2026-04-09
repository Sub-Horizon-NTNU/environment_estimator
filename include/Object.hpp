
#pragma once
#include "KalmanFilter.hpp"
#include "USVStates.hpp"
#include <memory>
#include <object_msgs/msg/object.hpp>
#include <chrono>


class Object {
    public:

    Object(const object_msgs::msg::Object::SharedPtr &object)
    : object_(object), prev_time_(std::chrono::steady_clock::now()),
        position_x_(object->position_x),
        position_y_(object->position_y)
        {

        Eigen::MatrixXd A;
        A.setIdentity(4,4);

        Eigen::MatrixXd H(4,4);
        H << 
        1.0,0.0,0.0,0.0, 
        0.0,1.0,0.0,0.0,
        0.0,0.0,1.0,0.0,
        0.0,0.0,0.0,1.0;

        Eigen::MatrixXd P;
        P.setIdentity(4,4);
        Eigen::VectorXd x(4);
        x << object->position_x,object->position_y,object->velocity_x,object->velocity_y;
        
        KF_ = std::make_shared<KalmanFilter>(x,A,H,P);
      }

    //Seconds
    double get_time_since_updated(){
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now-prev_time_).count();
        return dt;
    }

    void update(const object_msgs::msg::Object::SharedPtr &object){

        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now-prev_time_).count();

        double vx = (object->position_x-position_x_)/dt;
        double vy = (object->position_y-position_y_)/dt;

        position_x_ = object->position_x;
        position_y_ = object->position_y;

        Eigen::VectorXd z(4);
        z << object->position_x, object->position_y, vx, vy;
        KF_->set_measurement(z);

        Eigen::MatrixXd A(4,4);
        A << 
        1.0, 0.0,  dt, 0.0, //x 
        0.0, 1.0, 0.0,  dt, //y
        0.0, 0.0, 1.0, 0.0, //vx
        0.0, 0.0, 0.0, 1.0; //vy
        KF_->set_model(A);

        //TODO achieve better values for process and
        //measurements cov matrix by using the variance of the heading 
        //and distance to the object.

        Eigen::MatrixXd R(4, 4);
        R <<
        0.5, 0.0, 0.0, 0.0,
        0.0, 0.5, 0.0, 0.0,
        0.0, 0.0, 0.5, 0.0,
        0.0, 0.0, 0.0, 0.5;
       
        KF_->set_measurement_noise_cov(R);

        Eigen::MatrixXd Q(4, 4);
        Q <<
        0.1, 0.0, 0.0, 0.0,
        0.0, 0.1, 0.0, 0.0,
        0.0, 0.0, 0.05, 0.0,
        0.0, 0.0, 0.0, 0.05;
        KF_->set_process_noise_cov(Q);

        KF_->update();

        object_->position_x = KF_->get_estimates()(0);
        object_->position_y = KF_->get_estimates()(1);
        object_->velocity_x = KF_->get_estimates()(2);
        object_->velocity_y = KF_->get_estimates()(3);
        prev_time_ = now;
    }


    object_msgs::msg::Object get_predicted_position(){
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now-prev_time_).count();
        //Use constant velocity model to predict the objects position
        Eigen::MatrixXd A(4,4);
        A << 
        1.0, 0.0,  dt, 0.0, //x 
        0.0, 1.0, 0.0,  dt, //y
        0.0, 0.0, 1.0, 0.0, //vx
        0.0, 0.0, 0.0, 1.0; //vy
//
        KF_->set_model(A);
        KF_->predict();
        Eigen::VectorXd predicted = KF_->get_predictions();
//
        object_msgs::msg::Object predicted_object;
        predicted_object.position_x = predicted(0);
        predicted_object.position_y = predicted(1);
        predicted_object.velocity_x = predicted(2);
        predicted_object.velocity_y = predicted(3);
//
        return predicted_object;
    }

    object_msgs::msg::Object::SharedPtr get() const {
        return object_;
    }
    
    double get_radius() const {
        return radius_;
    }

    std::chrono::steady_clock::time_point prev_time_;

    object_msgs::msg::Object::SharedPtr object_;
    double position_x_;
    double position_y_;
    double radius_;

    std::shared_ptr<KalmanFilter> KF_;

};