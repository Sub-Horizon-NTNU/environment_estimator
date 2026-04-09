from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    fov_arg = DeclareLaunchArgument('field_of_view', default_value='78.0', description= "Field of view for the usv in degrees")
    max_radius_arg = DeclareLaunchArgument('max_radius', default_value='20.0', description= "Max detection radius in [m]")
    min_radius_arg = DeclareLaunchArgument('min_radius', default_value='0.75', description= "Min detection radius in [m]")
    
    node =  Node(
        package='environment_estimator',
        executable='environment_estimator',
        name='environment_estimator',
        output='screen',
        parameters=[
            {"field_of_view":LaunchConfiguration("field_of_view")},
            {"max_radius":LaunchConfiguration("max_radius")},
            {"min_radius":LaunchConfiguration("min_radius")}
        ]
    )

    return LaunchDescription([
        fov_arg,
        max_radius_arg,
        min_radius_arg,
        node 
    ])
