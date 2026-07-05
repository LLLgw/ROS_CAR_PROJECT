import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown


def generate_launch_description():

    pkg_name = 'robot_cartographer'

    pkg_share = FindPackageShare(package =pkg_name).find(pkg_name)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(pkg_share, 'config') )
    configuration_basename = LaunchConfiguration('configuration_basename', default='robot_2d.lua')
    ekf_config = os.path.join(pkg_share,'config','carto_ekf.yaml')      #ekf配置文件
    

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
        ('/odom', '/odom_combined')],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename])

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])


    #卡尔曼滤波
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='carto_ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('/odometry/filtered', '/odom_combined')
        ]
    )


    ld = LaunchDescription()
    ld.add_action(ekf_node)
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)
    

    return ld