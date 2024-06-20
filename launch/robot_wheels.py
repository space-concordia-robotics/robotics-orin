import os
import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.actions import LogInfo

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    wheels_controller_driver = LifecycleNode(
        package='wheels_controller',
        executable='wheels_controller_node',
        name='wheels_controller_node',
        output='screen',
        parameters=[
            {'multiplier': 2000},
            {'local_mode': True}
        ],
        namespace='/',
    )

    configure_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=wheels_controller_driver, goal_state='configured',
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch] wheels controller node is configuring."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(wheels_controller_driver),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )


    urdf_file_name = 'astro_arm.urdf'
    urdf = os.path.join(
        get_package_share_directory('arm_ik'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Node(
        #     package='arm_ik',
        #     executable='CadMouseJoyNode',
        #     name='cad_mouse_joy_node',
        #     output='screen',
        #     parameters=[
        #         # More deadzone on yaw (pivot)
        #         {'deadzones': [20, 20, 20, 20, 20, 200]}
        #     ]
        # ),
        wheels_controller_driver,
        configure_event,
        LifecycleNode(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            namespace="/"
        ),
    ])
