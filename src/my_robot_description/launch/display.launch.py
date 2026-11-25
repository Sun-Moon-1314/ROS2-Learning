import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # 找到功能包安装后的 share 目录
    package_path = get_package_share_directory(
        "my_robot_description"
    )

    # 拼接 URDF 文件路径
    urdf_path = os.path.join(
        package_path,
        "urdf",
        "my_robot.urdf.xacro"
    )
    rviz_config_path = os.path.join(
            package_path,
            "rviz",
            "urdf_config.rviz"
        )

    # 读取并处理 URDF/Xacro 文件
    robot_description = ParameterValue(
        Command(["xacro ", urdf_path]),
        value_type=str
    )

    # 发布机器人模型和 TF 坐标
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # 发布关节状态
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    # 启动 RViz2
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    # 真正启动上面定义的节点
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node
    ])
