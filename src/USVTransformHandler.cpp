
#include "USVTransformHandler.hpp"



    USVTransformHandler::USVTransformHandler(rclcpp::Node::SharedPtr node):
    node_(node),
    heading_(0.0),
    pitch_(0.0),
    roll_(0.0)
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        get_pose_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(40),
            std::bind(&USVTransformHandler::update_usv_position, this));

    }

    object_msgs::msg::Object USVTransformHandler::camera_to_world(const object_msgs::msg::Object &object_camera_coordinates){
        //Tranforms vector in the camera reference frame to world NED.

        object_msgs::msg::Object object;
        geometry_msgs::msg::PointStamped camera_coordinates;
        geometry_msgs::msg::PointStamped world_coordinates;
        camera_coordinates.header.stamp = object_camera_coordinates.header.stamp;
        camera_coordinates.header.frame_id = "camera"; //object_camera_coordinates.header; // frame and time, camera frame is called "camera"
        camera_coordinates.point.x = object_camera_coordinates.position_x;
        camera_coordinates.point.y = object_camera_coordinates.position_y;
        camera_coordinates.point.z = object_camera_coordinates.position_z;
        try {
            //auto t = tf_buffer_->lookupTransform("world_ned", "usv_ned", tf2::TimePointZero);
            //RCLCPP_INFO(node_->get_logger(), "usv pos in world: %.2f %.2f %.2f",
            //    t.transform.translation.x,
            //    t.transform.translation.y,
            //    t.transform.translation.z);
            tf_buffer_->transform(
                camera_coordinates,
                world_coordinates, 
                "world_ned",
                tf2::durationFromSec(0.1)
            );

            object.header.stamp = node_->now();
            object.color = object_camera_coordinates.color;
            object.type = object_camera_coordinates.type;
            object.id = object_camera_coordinates.id;
            object.position_x = world_coordinates.point.x; // "world_ned" 
            object.position_y = world_coordinates.point.y; // "world_ned" 
            object.position_z = world_coordinates.point.z; // "world_ned"   
            return object;
        }
        catch(const tf2::TransformException &tf_ex){
            RCLCPP_WARN(node_->get_logger(),"Failure when performing trasnform: %s",tf_ex.what());
            return object;
        }
        
        }
    
    void USVTransformHandler::update_usv_position(){
        geometry_msgs::msg::TransformStamped transform;
        try{
            transform = tf_buffer_->lookupTransform(
                "world_ned",
                "usv_ned",
                tf2::TimePointZero,
                tf2::durationFromSec(0.1)); // Timeout
            pose_= transform.transform;
        } catch(const tf2::TransformException &tf_ex){
            RCLCPP_WARN(node_->get_logger(),"Failure to get transform from world to USV: %s", tf_ex.what());
            return;
        }

        tf2::Quaternion quat_tf;
        geometry_msgs::msg::Quaternion quat_msg = pose_.rotation;
        tf2::fromMsg(quat_msg, quat_tf);

        tf2::Matrix3x3 m_rot(quat_tf);
        m_rot.getRPY(roll_, pitch_, heading_);
    }

    geometry_msgs::msg::Transform USVTransformHandler::get_usv_position(){
        return pose_;
    }

    double USVTransformHandler::get_heading(){
        return heading_;
    }
    
