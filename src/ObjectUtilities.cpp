#include "ObjectUtilities.hpp"
#include "USVTransformHandler.hpp"

    ObjectUtilities::ObjectUtilities(rclcpp::Node::SharedPtr node,const std::shared_ptr<USVTransformHandler> usv_transform_handler):
    node_(node),
    usv_transform_handler_(usv_transform_handler)
    {
        node->declare_parameter<double>("field_of_view", 78.0);
        node->declare_parameter<double>("max_radius", 20.0);
        node->declare_parameter<double>("min_radius", 1.0);

        field_of_view_ = node_->get_parameter("field_of_view").as_double();
        max_radius_ = node_->get_parameter("max_radius").as_double();
        min_radius_ = node_->get_parameter("min_radius").as_double();
    }

    object_msgs::msg::Object ObjectUtilities::transform_object(const object_msgs::msg::Object object_sensor_frame) {
        double cos_h = std::cos(usv_transform_handler_->get_heading());
        double sin_h = std::sin(usv_transform_handler_->get_heading());

        Eigen::Vector2d usv_pos(usv_transform_handler_->get_usv_position().translation.x,
                                usv_transform_handler_->get_usv_position().translation.y);

        Eigen::Matrix<double, 2, 2> T_object_boat;
        T_object_boat << 
        cos_h, -sin_h, 
        sin_h, cos_h;

        Eigen::Vector2d rel_pos;
        rel_pos << object_sensor_frame.position_x, object_sensor_frame.position_y;

        Eigen::Vector2d world_pos = T_object_boat * rel_pos + usv_pos;

        object_msgs::msg::Object object_world;

        object_world.position_x = world_pos.x();
        object_world.position_y = world_pos.y();
        object_world.type = object_sensor_frame.type;
        object_world.color = object_sensor_frame.color;
        object_world.id = object_sensor_frame.id;
        
        return object_world;
    }

    bool ObjectUtilities::should_be_visible(const object_msgs::msg::Object &object){
        return is_inside_radius(object) && is_inside_fov(object);
    }

    //double ObjectUtilities::get_accuracy(const object_msgs::msg::Object::SharedPtr &object){
    //    // https://static.generation-robots.com/media/stereolabs-zed-2i-datasheet.pdf
    //    double accuracy{};
    //    double dist = std::hypot(usv_states_->get_states().x-object->position_x,usv_states_->get_states().y-object->position_y);
    //    if(dist < 3.0){accuracy= 3*0.01; } else {
    //        accuracy = dist*0.07;
    //    }
    //
    //    
    //    return accuracy;
    //}
 
    bool ObjectUtilities::is_inside_radius(const object_msgs::msg::Object &object){
        double dist = std::hypot(usv_transform_handler_->get_usv_position().translation.x-object.position_x,usv_transform_handler_->get_usv_position().translation.y-object.position_y);
        if( (dist > max_radius_) || (dist < min_radius_) ){
            //RCLCPP_INFO(node_->get_logger(),"distance: %.2f",dist);
            return false;
        }
        return true;
    }

    bool ObjectUtilities::is_inside_fov(const object_msgs::msg::Object &object){
        double x_diff = object.position_x- usv_transform_handler_->get_usv_position().translation.x;
        double y_diff = object.position_y- usv_transform_handler_->get_usv_position().translation.y;

        double angle_diff = usv_transform_handler_->get_heading()-std::atan2(y_diff,x_diff);
        while (angle_diff >  M_PI){angle_diff -= 2.0 * M_PI; }
        while (angle_diff < -M_PI){angle_diff += 2.0 * M_PI; }
        
        if(std::abs(angle_diff*180/M_PI) < field_of_view_/2.0){ // divided by two to due to the range being in [-fov,fov]
            //RCLCPP_INFO(node_->get_logger(),"Angle: %.2f",angle_diff*180/M_PI);
            return true;
        }

        return false;
    }