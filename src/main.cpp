#include <rclcpp/rclcpp.hpp>

#include <memory>
#include "EnvironmentEstimatorNode.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<EnvironmentEstimatorNode> environment_estimator_node = std::make_shared<EnvironmentEstimatorNode>();
    environment_estimator_node->init();
    rclcpp::spin(environment_estimator_node);
    rclcpp::shutdown();
    return 0;
}
