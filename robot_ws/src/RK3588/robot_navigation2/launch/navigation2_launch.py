import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    #定位包
    robot_navigation2_dir = get_package_share_directory('robot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    #声明参数,获取配置文件
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_path = LaunchConfiguration('map',default=os.path.join(robot_navigation2_dir,'maps','my_map.yaml'))
    nav2_param_path = LaunchConfiguration('params_file',default=os.path.join(robot_navigation2_dir,'param','robot_nav2.yaml'))
    ekf_config = os.path.join(robot_navigation2_dir,'config','ekf_nav.yaml')

    #卡尔曼滤波
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='nav_ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', 'odom_combined')
        ]
    )

    nav2_bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir,'/launch','/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
    )




    return LaunchDescription([ekf_node,nav2_bringup_launch])