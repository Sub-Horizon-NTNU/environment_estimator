

#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <object_msgs/msg/object.hpp>
#include <geometry_msgs/msg/vector3.hpp>

class USVTransformHandler{
    public:
    USVTransformHandler(rclcpp::Node::SharedPtr node):
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

    object_msgs::msg::Object camera_to_world(const object_msgs::msg::Object &object_camera_coordinates){
        //Tranforms vector in the camera reference frame to world NED.

        object_msgs::msg::Object object;
        geometry_msgs::msg::Vector3Stamped camera_coordinates;
        geometry_msgs::msg::Vector3Stamped world_coordinates;
        camera_coordinates.header.stamp = object_camera_coordinates.header.stamp;
        camera_coordinates.header.frame_id = "camera"; //object_camera_coordinates.header; // frame and time, camera frame is called "camera"
        camera_coordinates.vector.x = object_camera_coordinates.position_x;
        camera_coordinates.vector.y = object_camera_coordinates.position_y;
        camera_coordinates.vector.z = object_camera_coordinates.position_z;
        try {
            tf_buffer_->transform(
                camera_coordinates,
                world_coordinates, 
                "world_ned",
                tf2_ros::fromMsg(camera_coordinates.header.stamp), 
                "world_ned",
                tf2::durationFromSec(0.1)
            );

            object.header.stamp = node_->now();
            object.color = object_camera_coordinates.color;
            object.type = object_camera_coordinates.type;
            object.id = object_camera_coordinates.id;
            object.position_x = world_coordinates.vector.x; // "world_ned" 
            object.position_y = world_coordinates.vector.y; // "world_ned" 
            object.position_z = world_coordinates.vector.z; // "world_ned"   
            return object;
            }
        catch(const tf2::TransformException &tf_ex){
            RCLCPP_WARN(node_->get_logger(),"Failure when performing trasnform: %s",tf_ex.what());
            return object;
        }
        
        }
    
    void update_usv_position(){
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

    geometry_msgs::msg::Transform get_usv_position(){
        return pose_;
    }

    double get_heading(){
        return heading_;
    }
    
    private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr get_pose_timer_;
    geometry_msgs::msg::Transform pose_{};
    double heading_;
    double pitch_;
    double roll_;
};

