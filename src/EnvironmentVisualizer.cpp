#include <EnvironmentVisualizer.hpp>

    
    EnvironmentVisualizer::EnvironmentVisualizer(rclcpp::Node::SharedPtr node,const std::shared_ptr<USVStates> usv_states, const std::shared_ptr<ObjectManager> object_manager): 
    node_(node), usv_states_(usv_states), object_manager_(object_manager) {
        marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
                    "selene/environment_estimator/buoy_markers", 10);

        usv_marker_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>(
                    "selene/environment_estimator/usv_marker", 10);

        marker_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&EnvironmentVisualizer::publish_markers, this));

        usv_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&EnvironmentVisualizer::publish_usv_marker, this));
    }


    void EnvironmentVisualizer::publish_markers(){
    visualization_msgs::msg::MarkerArray marker_array;
     // Clear all previous markers
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.header.frame_id = "map";
        delete_marker.header.stamp = node_->now();
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);

    for(unsigned int i = 0; i < object_manager_->get_objects().size(); i++){
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = node_->now();
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position.x = object_manager_->get_objects()[i]->get()->position_y; // coordinate axis swapped
        marker.pose.position.y = object_manager_->get_objects()[i]->get()->position_x;
        marker.pose.position.z = 0.0;

        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;

        marker.color.a = 1.0;
        // Set color based on object color string
        if(object_manager_->get_objects()[i]->get()->color[0] == 'r'){
            marker.color.r = 1.0;
        } else if(object_manager_->get_objects()[i]->get()->color[0] == 'g'){
            marker.color.g = 1.0;
        } else if(object_manager_->get_objects()[i]->get()->color[0] == 'y'){
            marker.color.r = 1.0; marker.color.g = 1.0;
        } else {
            marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 1.0;
        }

        marker_array.markers.push_back(marker);

        object_msgs::msg::Object predicted = object_manager_->get_objects()[i]->get_predicted_position();
        visualization_msgs::msg::Marker predicted_marker;
        predicted_marker.header.frame_id = "map";
        predicted_marker.header.stamp = node_->now();
        predicted_marker.id = i + 1000;
        predicted_marker.type = visualization_msgs::msg::Marker::SPHERE;
        predicted_marker.action = visualization_msgs::msg::Marker::ADD;

        predicted_marker.pose.position.x = predicted.position_y;
        predicted_marker.pose.position.y = predicted.position_x;
        predicted_marker.pose.position.z = 0.0;

        predicted_marker.scale.x = 0.3;
        predicted_marker.scale.y = 0.3;
        predicted_marker.scale.z = 0.3;

        predicted_marker.color.a = 1.0;
        predicted_marker.color.r = 1.0;
        predicted_marker.color.g = 0.4;
        predicted_marker.color.b = 0.7;

        marker_array.markers.push_back(predicted_marker); 
    }
    marker_publisher_->publish(marker_array);
}

void EnvironmentVisualizer::publish_usv_marker(){
    visualization_msgs::msg::Marker usv_marker;
    usv_marker.header.frame_id = "map";
    usv_marker.header.stamp = node_->now();
    usv_marker.id = 0;
    usv_marker.type = visualization_msgs::msg::Marker::CUBE;
    usv_marker.action = visualization_msgs::msg::Marker::ADD;
    usv_marker.color.a = 1.0;
    usv_marker.color.b = 1.0;
    usv_marker.pose.orientation = usv_states_->get_orientation();

    usv_marker.pose.position.x = usv_states_->get_states().y; // coordinate axis swapped
    usv_marker.pose.position.y = usv_states_->get_states().x;
    usv_marker.pose.position.z = 0.0;
    usv_marker.scale.x = 0.8;
    usv_marker.scale.y = 1.0;
    usv_marker.scale.z = 0.5;
    usv_marker_publisher_->publish(usv_marker);

}