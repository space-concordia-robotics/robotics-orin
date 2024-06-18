import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='beep_autonomy').find('beep_autonomy')
    bringup_dir = launch_ros.substitutions.FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    zed2_dir=launch_ros.substitutions.FindPackageShare(package='zed_wrapper').find('zed_wrapper')
    ouster_dir=launch_ros.substitutions.FindPackageShare(package='ouster_ros').find('ouster_ros')
    slam_dir=launch_ros.substitutions.FindPackageShare(package='slam_toolbox').find('slam_toolbox')
    default_model_path = os.path.join(pkg_share, 'src/description/rover_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/default_view.rviz')
    default_params_file=os.path.join(pkg_share, 'config/nav2_params_no_map.yaml')
    world_path=os.path.join(pkg_share, 'world/sonoma.world')
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/test-ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
       remappings=[("odometry/filtered", "odometry/local")]
    )

    lidar_launch=launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [ouster_dir,'/launch/driver.launch.py']),
            launch_arguments={'params_file':os.path.join(pkg_share, 'config/ouster_driver_params.yaml'),
                                'viz':'false'}.items()
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                    description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                    description='Absolute path to robot urdf file'),
        robot_localization_node,        
        lidar_launch,
    ])
