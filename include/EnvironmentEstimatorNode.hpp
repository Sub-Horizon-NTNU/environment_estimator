#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ObjectManager.hpp>
#include "EnvironmentVisualizer.hpp"
#include "USVTransformHandler.hpp"

class EnvironmentEstimatorNode : public rclcpp::Node {
public:
    EnvironmentEstimatorNode();

    void init();
    
private:
    std::shared_ptr<USVTransformHandler> usv_transform_handler_;
    std::shared_ptr<ObjectManager> object_manager_;
    std::unique_ptr<EnvironmentVisualizer> environment_visualizer_;



};

