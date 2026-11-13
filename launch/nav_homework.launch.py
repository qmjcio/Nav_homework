import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 用 install 出来的 share 目录拼路径，换台电脑、换个用户名都不用改
    pkg_share = get_package_share_directory('nav_homework')

    map_yaml = LaunchConfiguration('map_yaml')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml',
            default_value=os.path.join(pkg_share, 'maps', 'mymap.yaml'),
            description='地图的 yaml 文件（和 pgm 在同一目录）'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'costmap_params.yaml'),
            description='costmap 参数文件'),

        # 静态 TF。costmap 内部更新线程要找 map->base_link，少了会一直报
        # "Could not get robot pose for costmap update"
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
            output='screen',
        ),

        # map_server 是 lifecycle 节点，启动时是 unconfigured，不会发 /map
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': map_yaml}],
            output='screen',
        ),

        # 等它起来，先 configure 再 activate。
        # activate 必须在上一步成功之后，所以套在 configure 内层，
        # 不能和 configure 并列 —— 并列会并发执行，activate 可能先跑然后被拒。
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
                    output='screen'),
                TimerAction(
                    period=1.5,
                    actions=[
                        ExecuteProcess(
                            cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
                            output='screen'),
                    ],
                ),
            ],
        ),

        Node(
            package='nav_homework',
            executable='nav_node',
            name='nav_node',
            parameters=[params_file],
            output='screen',
        ),

        # 带上配置，打开就有 Fixed Frame / Map / Path，不用手点
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'nav.rviz')],
            output='screen',
        ),
    ])
