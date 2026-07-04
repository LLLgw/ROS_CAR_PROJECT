import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, TimerAction
from controller_manager_msgs.srv import SwitchController
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
        pkg_name = 'robot_low_connect'
        rs_pkg_name = 'rslidar_sdk'

        ld = LaunchDescription()
        pkg_share = FindPackageShare(package = pkg_name).find(pkg_name)
        rviz_config=get_package_share_directory('rslidar_sdk')+'/rviz/rviz2.rviz'

        config_point = os.path.join(pkg_share, 'config', 'pointcloud_to_2d.yaml')

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

        ld.add_action(pointcloud_node)
        ld.add_action(rs_start_node)
        ld.add_action(rviz2_node)

        return ld
       