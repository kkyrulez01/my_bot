from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ros_gz_bridge.actions import RosGzBridge
from launch_ros.actions import Node

def generate_launch_description():

    package_name='my_bot' 

    # Create a bridge between ROS2 and Gazebo topics
    
    # CmdVel bridge
    gz_bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel' + '@geometry_msgs/msg/Twist' + '@gz.msgs.Twist',],
        output='screen'
    )
    # Odometry and Clock bridge
    gz_bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/odom' + '@nav_msgs/msg/Odometry' + '[gz.msgs.Odometry',
                    '/clock' + '@rosgraph_msgs/msg/Clock' + '[gz.msgs.Clock'],
        output='screen',
    )
    # TF bridge
    gz_bridge_tf = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/tf' + '@tf2_msgs/msg/TFMessage' + '[gz.msgs.Pose_V',],
        output='screen',
    )
    # Joint states bridge
    gz_bridge_joint_states = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/joint_states' + '@sensor_msgs/msg/JointState' + '[gz.msgs.Model',],
        output='screen',
    )
    # Laser scan bridge
    gz_bridge_scan = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan' + '@sensor_msgs/msg/LaserScan' + '[gz.msgs.LaserScan',],
        output='screen',
    )
    # Camera Image bridge
    gz_bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera/image_raw' + '@sensor_msgs/msg/Image' + '[gz.msgs.Image',],
    )
    # Camera Info bridge
    gz_bridge_camera_info = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera/camera_info' + '@sensor_msgs/msg/CameraInfo' + '[gz.msgs.CameraInfo',],
    )
    # Launch nodes
    return LaunchDescription([
        gz_bridge_cmd_vel,
        gz_bridge_odom,
        gz_bridge_tf,
        gz_bridge_joint_states,
        gz_bridge_scan,
        gz_bridge_camera,
        gz_bridge_camera_info,
    ])
    