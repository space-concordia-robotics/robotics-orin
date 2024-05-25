import os
import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.actions import LogInfo

def generate_launch_description():
    wheel_controller_driver = LifecycleNode(
        package='wheels_controller',
        executable='wheels_controller_node',
        name='wheels_controller',
        output='screen',
        parameters=[
            {'multiplier': 2000},
            {'local_mode': False}
        ],
        namespace='/',
    )
    
    arm_ik_driver = LifecycleNode(
        package='arm_ik',
        executable='IKNode',
        name='ik_node',
        output='screen',
        parameters=[
            {'joint_lengths': [1.354, 1.333, 1.250]},
            {'joint_angle_mins': [-180.0, -80.0, -111.0, -101.0]},
            {'joint_angle_maxes': [180, 80.0, 115.0, 106.0]},
            {'sensitivity': 1.0},
            {'mode': '2D'},
            {'solution': 1},
            # "joint" sets final joint angle, while "vertical" sets the
            # angle of the gripper relative to vertical while keeping the end effector
            # position constant.
            {'angle_set': 'vertical'},
            {'local_mode': False}
        ],
        namespace='/',
    )

    arm_controller_driver = LifecycleNode(
        package='arm_controller',
        executable='arm_controller_node',
        name='arm_controller',
        output='screen',
        namespace='/',
    )

    absenc_interface_driver = LifecycleNode(
        package='absenc_interface',
        executable='absenc_node',
        name='absenc_node',
        output='screen',
        namespace='/',
    )

    configure_wcd_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=wheel_controller_driver, goal_state='configuring',
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch]  node is configuring."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(wheel_controller_driver),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )
    configure_arid_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=arm_ik_driver, goal_state='configuring',
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch]  node is configuring."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(arm_ik_driver),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )
    configure_acd_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=arm_controller_driver, goal_state='configuring',
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch]  node is configuring."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(arm_controller_driver),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )
    configure_abid_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=absenc_interface_driver, goal_state='configuring',
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch]  node is configuring."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(absenc_interface_driver),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )



    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'astro_arm.urdf'
    urdf = os.path.join(
        get_package_share_directory('arm_ik'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # LifecycleNode(
        #     package='absenc_interface',
        #     executable='absenc_node',
        #     name='absenc_node',
        #     output='screen'
        # ),
        # LifecycleNode(
        #     package='arm_controller',
        #     executable='arm_controller_node',
        #     name='arm_controller',
        #     output='screen'
        # ),
        # LifecycleNode(
        #     package='arm_ik',
        #     executable='IKNode',
        #     name='ik_node',
        #     output='screen',
        #     parameters=[
        #         {'joint_lengths': [1.354, 1.333, 1.250]},
        #         {'joint_angle_mins': [-180.0, -80.0, -111.0, -101.0]},
        #         {'joint_angle_maxes': [180, 80.0, 115.0, 106.0]},
        #         {'sensitivity': 1.0},
        #         {'mode': '2D'},
        #         {'solution': 1},
        #         # "joint" sets final joint angle, while "vertical" sets the
        #         # angle of the gripper relative to vertical while keeping the end effector
        #         # position constant.
        #         {'angle_set': 'vertical'},
        #         {'local_mode': False}
        #     ]
        # ),
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
        wheel_controller_driver,
        arm_ik_driver,
        arm_controller_driver,
        absenc_interface_driver,
        configure_wcd_event,
        configure_arid_event,
        configure_acd_event,
        configure_abid_event,
        LifecycleNode(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            namespace='/'
        ),
    ])
