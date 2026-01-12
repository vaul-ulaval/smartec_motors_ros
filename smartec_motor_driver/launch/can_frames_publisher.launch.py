from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    
    ld = LaunchDescription()
    share_path = get_package_share_directory("smartec_motor_driver")
    
    # Load config files
    config = os.path.join(share_path, "config", "smartec_motor_driver.yaml")
    db_file = os.path.join(share_path, "config", "smartec_canbus.dbc")

    # init_canbus = ExecuteProcess(
    #     name="init_can",
    #     cmd=["sudo ip link set can32 up type can bitrate 500000"],
    #     shell=True
    # )
    # ld.add_action(init_canbus)

    can_frames_publisher = Node(
        package='smartec_motor_driver',
        executable='can_frames_publisher.py',
        name='can_frames_publisher',
        parameters=[
            config,
            {"database_file": db_file}
        ],
    )
    ld.add_action(can_frames_publisher)

    return ld