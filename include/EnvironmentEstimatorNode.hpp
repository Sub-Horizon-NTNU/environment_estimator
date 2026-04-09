#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ObjectManager.hpp>
#include "EnvironmentVisualizer.hpp"
#include "USVStates.hpp"

class EnvironmentEstimatorNode : public rclcpp::Node {
public:
    EnvironmentEstimatorNode();

    void init();
    
private:
    std::shared_ptr<USVStates> usv_states_;
    std::shared_ptr<ObjectManager> object_manager_;
    std::unique_ptr<EnvironmentVisualizer> environment_visualizer_;



};

