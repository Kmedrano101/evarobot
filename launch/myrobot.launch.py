import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    package_dir = get_package_share_directory('myrobot')
    # Define the URDF file paths
    urdf_file = os.path.join(package_dir, 'urdf', 'my_robot.urdf')
    gz_world_file = os.path.join(package_dir, 'urdf', 'robot_wheeled.sdf')
    
    # Launch Gazebo Garden using `gz sim`
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', gz_world_file],
        prefix='gnome-terminal --',
    )

    return LaunchDescription([

        DeclareLaunchArgument('world', default_value=gz_world_file, description='Gazebo world file'),
        gz_sim,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='myrobot',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_ros_bridge',
            output='screen',
            parameters=[
                {'config_file': os.path.join(package_dir, 'config', 'gz_ros_bridge.yaml')}
            ],
        ),
        Node(
            package='myrobot',
            executable='tf_publisher.py',
            name='tf_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='myrobot',
            executable='frame_manager.py',
            name='frame_manager',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='myrobot',
            executable='lidar_transform',
            name='lidar_transform',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(package_dir,'config', 'rviz_config.rviz')]]
        )
    ])