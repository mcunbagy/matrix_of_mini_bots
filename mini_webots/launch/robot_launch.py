import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
#from webots_ros2_driver.webots_launcher import Ros2SupervisorLauncher


def generate_launch_description():

    use_camera = LaunchConfiguration('camera', default=True)

    #ros2_supervisor = Ros2SupervisorLauncher()


    package_dir = get_package_share_directory('mini_webots')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'my_robot.urdf')).read_text()
    #ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control_configuration.yml')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world.wbt')
    )

    

    my_robot_driver = Node(
        package='webots_ros2_driver',
        name='driver',
        executable='driver',
        additional_env={'WEBOTS_ROBOT_NAME': 'minaros'},
        output='screen',
        parameters=[
           {'robot_description': robot_description,
             'set_robot_state_publisher': False},
             #ros2_control_params
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    # camera = Node(
    #     package='mini_webots',
    #     node_executable='camera',
    #     output='screen',
    #     condition=launch.conditions.IfCondition(use_camera)
    # )


    return LaunchDescription([
        webots,
        my_robot_driver,
        robot_state_publisher,
        #camera,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
