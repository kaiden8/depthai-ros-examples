from launch_ros.actions import Node, ComposableNodeContainer

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import LogInfo
import launch_ros.descriptions

def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    print(ThisLaunchFileDir())
    # dai_launch_dir = ThisLaunchFileDir()

    # lg = LogInfo(msg=[
    #         'Including launch file located at: ', ThisLaunchFileDir(), '/dai_robot.launch.py'])
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    
    nav2_param_path = os.path.join(
            get_package_share_directory('dai_turtlebot3_description'),
            'param',
            param_file_name)
    
    nav2_param_dir = LaunchConfiguration(
        'nav2_params_file',
        default=nav2_param_path)

        # Create the launch configuration variables
    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')

    #TODO(Sachin): Do I need to remap this ?
    """ remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')] """

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'turtlebot3_world.yaml'),
        description='Full path to map file to load')


    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value = nav2_param_dir,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', 'follow_point.xml'),
        description='Full path to the behavior tree xml file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')
    print("The second------------")
    print(nav2_param_path)
    # print(ThisLaunchFileDir().perform)
    
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': 'false',
                          'params_file': params_file,
                          'default_bt_xml_filename': default_bt_xml_filename,
                          'autostart': autostart}.items())

    pointCloud_converter = ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::ConvertMetricNode',
                    name='convert_metric_node',
                    remappings=[('image_raw', '/stereo/depth'),
                                ('camera_info', '/stereo/camera_info'),
                                ('image', '/stereo/converted_depth')]
                ),
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyziNode',
                    name='point_cloud_xyzi',

                    remappings=[('depth/image_rect', '/stereo/converted_depth'),
                                ('intensity/image_rect', '/right/image'),
                                ('intensity/camera_info', '/right/camera_info'),
                                ('points', '/stereo/points')]
                ),
            ],
            output='screen',
        )
            # prefix=['xterm -e gdb -ex run --args'],

    pcl_to_scan_cmd = launch_ros.actions.Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan_node',
            output='screen',
            parameters=[{'target_frame': "oak-d_right_camera_frame"},
                        {'range_min': 0.7},
                        {'range_max': 7.2}],
            remappings=[('cloud_in','/stereo/points')])
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')))

    """ turtlebot_rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/turtlebot_rviz.launch.py'])) """
    """     
    depth_to_scan = Node(
            package='depthimage_to_laserscan',
            node_executable='depthimage_to_laserscan_node',
            node_name='depthimage_to_laserscan_node',
            output='screen',
            parameters=[{'output_frame': output_frame},
                        {'range_min': range_min},
                        {'range_max': range_max}],
            arguments=['depth:=/stereo/depth',
                       'depth_camera_info:=/stereo/camera_info',
                       'scan:=/scan']) """

    ld = LaunchDescription()
    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_autostart_cmd)
    # ld.add_action(lg)

    # ld.add_action(bringup_cmd)
    ld.add_action(pointCloud_converter)
    ld.add_action(pcl_to_scan_cmd)
    # ld.add_action(rviz_cmd)
    return ld
