import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    config_file = 'slam_toolbox_online_async.yaml'
    config_dir = os.path.join(get_package_share_directory('localization_server'),'config',config_file)

    slam = ExecuteProcess(
        cmd=[[
            'ros2 launch andino_gz slam_toolbox_online_async.launch.py ',
            'slam_params_file:=',
            config_dir,
        ]],
        shell=True
    )

    rviz_file = 'localization.rviz'
    rviz_dir = os.path.join(get_package_share_directory('localization_server'),'rviz',rviz_file)

    rviz_node = Node(
    
        package='rviz2',
        executable='rviz2',
        name='rviz_node',
        arguments=['-d', os.path.join(get_package_share_directory('localization_server'),'rviz',rviz_dir)]
    
    )

    return LaunchDescription([
    
        slam,

    ])
