#include "EnvironmentEstimatorNode.hpp"

    EnvironmentEstimatorNode::EnvironmentEstimatorNode() : Node("environment_estimator"){}

    void EnvironmentEstimatorNode::init(){
        usv_states_ = std::make_shared<USVStates>(this->shared_from_this());
        object_manager_ = std::make_shared<ObjectManager>(this->shared_from_this(),usv_states_);
        environment_visualizer_ = std::make_unique<EnvironmentVisualizer>(this->shared_from_this(),usv_states_,object_manager_);
        RCLCPP_INFO(this->get_logger(),"Environment Estimator started");
    }


