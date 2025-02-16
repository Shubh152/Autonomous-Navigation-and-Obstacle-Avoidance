import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='diff_car_model').find('diff_car_model')
    default_model_path = os.path.join(pkg_share, 'src/model/diff_car_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/nav2_config.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])},{'use_sim_time' : True}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path])},{'use_sim_time' : True}],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig'), "--ros-args", "--remap", "use_sim_time:=true"],
    )

    pkg_nav2 = get_package_share_directory('nav2_bringup')
    nav2_launch_path = PathJoinSubstitution([pkg_nav2, 'launch', 'bringup_launch.py'])
    # ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/shubh/diff_drive_car/diff_car_model/config/slam_toolbox.yaml


    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # pkg_spaceros_gz_sim = get_package_share_directory('spaceros_gz_sim')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    gz_model_path = PathJoinSubstitution([pkg_share, 'src/model'])
    gz_world_path = PathJoinSubstitution([gz_model_path, 'car_world.sdf'])

    ros_gz_bridge_lidar = launch_ros.actions.Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=["/base_lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"],
            remappings=[],
            output='screen',
            parameters=[{'use_sim_time' : True}]
        )

    ros_gz_bridge_imu = launch_ros.actions.Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=["/base_imu@sensor_msgs/msg/Imu@gz.msgs.IMU"],
            remappings=[],
            output='screen',
            parameters=[{'use_sim_time' : True}]
        )
    
    ros_gz_bridge_odom = launch_ros.actions.Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=["model/sam_bot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry"],
            remappings=[],
            output='screen',
            parameters=[{'use_sim_time' : True}]
        )
    
    ros_gz_bridge_tf = launch_ros.actions.Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=["--ros-args", "-p", "config_file:=/home/shubh/diff_drive_car/diff_car_model/config/ros_gz_bridge_tf.yaml"],
            remappings=[],
            output='screen',
            parameters=[{'use_sim_time' : True}]
        )
    
    ros_gz_bridge_clock = launch_ros.actions.Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=["--ros-args", "-p", "config_file:=/home/shubh/diff_drive_car/diff_car_model/config/ros_gz_clock.yaml"],
            remappings=[],
            output='screen',
            parameters=[{'use_sim_time' : True}]
        )
    
    ros_gz_bridge_teleop = launch_ros.actions.Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=["cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist"],
            remappings=[],
            output='screen',
            parameters=[{'use_sim_time' : True}]
        )

    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf.yaml'),{'use_sim_time' : True}]
    )

    broadcast_tf_static_lidar = launch_ros.actions.Node(
       package='tf2_ros',
       executable='static_transform_publisher',
       name='static_transform_publisher',
       output='screen',
       arguments=["0", "0", "0", "0", "0", "0", "lidar_link", "sam_bot/base_link/lidar"],
        parameters=[{'use_sim_time' : True}]
    )

    broadcast_tf_static_imu = launch_ros.actions.Node(
       package='tf2_ros',
       executable='static_transform_publisher',
       name='static_transform_publisher',
       output='screen',
       arguments=["0", "0", "0", "0", "0", "0", "base_link", "sam_bot/base_link/imu_sensor"],
            parameters=[{'use_sim_time' : True}]
    )


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),

        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),

        launch.actions.DeclareLaunchArgument(
            'world',
            default_value=gz_world_path,
            description='World to load into Gazebo'
        ),
        launch.actions.SetLaunchConfiguration(name='world_file', value=[LaunchConfiguration('world')]),
        launch.actions.SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_model_path),
        launch.actions.IncludeLaunchDescription( 
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': ['-r ',LaunchConfiguration('world_file')],
                'on_exit_shutdown': 'True'
            }.items(),
        ),
        launch.actions.IncludeLaunchDescription( 
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'use_sim_time' : "True",
                'use_localization' : "True",
                'params_file' : '/home/shubh/diff_drive_car/diff_car_model/config/nav2_params_diff.yaml'
            }.items(),
        ),
        joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        ros_gz_bridge_lidar,
        ros_gz_bridge_imu,
        ros_gz_bridge_odom,
        ros_gz_bridge_tf,
        ros_gz_bridge_teleop,
        ros_gz_bridge_clock,
        robot_localization_node,
        broadcast_tf_static_lidar,
        broadcast_tf_static_imu,
    ])