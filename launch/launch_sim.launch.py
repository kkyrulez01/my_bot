import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ros_gz_bridge.actions import RosGzBridge

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='my_bot' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gz_sim = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': '-r empty.sdf'}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', '/robot_description',
                                   '-name', 'my_bot',],
                        output='screen')

    # Create a bridge between ROS2 and Gazebo topics
    gz_bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel' + '@geometry_msgs/msg/Twist' + '@gz.msgs.Twist',],
        output='screen'
    )

    gz_bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/odom' + '@nav_msgs/msg/Odometry' + '[gz.msgs.Odometry',
                    '/clock' + '@rosgraph_msgs/msg/Clock' + '[gz.msgs.Clock'],
        output='screen',
    )
    
    gz_bridge_tf = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/tf' + '@tf2_msgs/msg/TFMessage' + '[gz.msgs.Pose_V',],
        output='screen',
    )

    gz_bridge_joint_states = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/joint_states' + '@sensor_msgs/msg/JointState' + '[gz.msgs.Model',],
        output='screen',
    )
    # Launch them all!
    return LaunchDescription([
        rsp,
        gz_sim,
        spawn,
        gz_bridge_cmd_vel,
        gz_bridge_odom,
        gz_bridge_tf,
        gz_bridge_joint_states,
    ])