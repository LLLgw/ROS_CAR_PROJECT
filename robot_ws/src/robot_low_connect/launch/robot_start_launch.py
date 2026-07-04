import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_name_in_model = 'robot'           #模型名称
    package_name = 'robot_low_connect'      #包名
    urdf_name = 'robot.urdf'                #模型文件
    rs_pkg_name = 'rslidar_sdk'

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    
    robot_config = os.path.join(pkg_share,'config','robot_param.yaml')   #robot配置文件
    rviz_config=get_package_share_directory('rslidar_sdk')+'/rviz/rviz2.rviz'
    config_point = os.path.join(pkg_share, 'config', 'pointcloud_to_2d.yaml')

    # Start Robot State publisher
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path]
    )

    #imu
    H30_node = Node(
        package = 'robot_low_connect',
        executable = 'H30_node',
        name = 'H30_node',
        output='screen'
    )

    #bottom_communication
    robot_info_transform = Node(
        package = 'robot_low_connect',
        executable = 'robot_info_transform',
        name = 'robot_info_transform',
        parameters = [robot_config],
        output='screen'
    )

    #gps
    G60_node = Node(
        package = 'robot_low_connect',
        executable = 'G60_node',
        name = 'G60_node',
        output='screen'
    )

    #start ladir
    rs_start_node = Node(
        package = rs_pkg_name,
        executable = 'rslidar_sdk_node',
        name = 'rslidar_sdk_node',
        output='screen'
    )

    #point_to_2d
    pointcloud_node = Node(
        package = 'pointcloud_to_laserscan',
        executable = 'pointcloud_to_laserscan_node',
        name = 'pointcloud_to_laserscan_node',
        parameters = [config_point],
        remappings = [
            ('cloud_in','/rslidar_points'),
            ('scan',"/scan")
        ]
    )


    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d',rviz_config],
        output='screen'
    )

    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(robot_info_transform)
    ld.add_action(H30_node)
    ld.add_action(rs_start_node)
    ld.add_action(pointcloud_node)
    ld.add_action(G60_node)
    ld.add_action(rviz2_node)

    return ld