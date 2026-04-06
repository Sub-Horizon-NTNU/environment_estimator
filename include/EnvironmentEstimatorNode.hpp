#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ObjectManager.hpp>
#include "EnvironmentVisualizer.hpp"
#include "USVStates.hpp"

class EnvironmentEstimatorNode : public rclcpp::Node {
public:
    EnvironmentEstimatorNode() : Node("environment_estimator")
    {   
             
    }

    void init(){
        usv_states_ = std::make_shared<USVStates>(this->shared_from_this());
        object_manager_ = std::make_shared<ObjectManager>(this->shared_from_this(),usv_states_);
        environment_visualizer_ = std::make_unique<EnvironmentVisualizer>(this->shared_from_this(),usv_states_,object_manager_);
    }
    
private:
    std::shared_ptr<USVStates> usv_states_;
    std::shared_ptr<ObjectManager> object_manager_;
    std::unique_ptr<EnvironmentVisualizer> environment_visualizer_;



};

