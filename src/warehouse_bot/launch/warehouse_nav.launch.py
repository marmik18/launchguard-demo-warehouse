import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

os.environ['TURTLEBOT3_MODEL'] = 'burger'
os.environ['GAZEBO_MODEL_DATABASE_URI'] = ''

SPAWN_X = '0.0'
SPAWN_Y = '-2.0'
SPAWN_Z = '0.01'
SPAWN_YAW = '0.0'


def generate_launch_description():
    pkg_warehouse_bot = get_package_share_directory('warehouse_bot')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_warehouse_world = get_package_share_directory(
        'aws_robomaker_small_warehouse_world'
    )

    models_dir = os.path.join(pkg_turtlebot3_gazebo, 'models')
    warehouse_models = os.path.join(pkg_warehouse_world, 'models')
    existing = os.environ.get('GAZEBO_MODEL_PATH', '')
    os.environ['GAZEBO_MODEL_PATH'] = ':'.join(filter(None, [
        models_dir,
        warehouse_models,
        pkg_warehouse_world,
        '/usr/share/gazebo-11/models',
        existing,
    ]))

    world = os.path.join(
        pkg_warehouse_world, 'worlds', 'small_warehouse', 'small_warehouse.world',
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    urdf_path = os.path.join(
        pkg_turtlebot3_gazebo, 'urdf', 'turtlebot3_burger.urdf',
    )
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
        }],
        output='screen',
    )

    sdf_path = os.path.join(
        pkg_turtlebot3_gazebo, 'models', 'turtlebot3_burger', 'model.sdf',
    )
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_burger',
            '-file', sdf_path,
            '-x', SPAWN_X, '-y', SPAWN_Y, '-z', SPAWN_Z, '-Y', SPAWN_YAW,
        ],
        output='screen',
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': os.path.join(pkg_warehouse_bot, 'maps', 'warehouse.yaml'),
            'params_file': os.path.join(pkg_warehouse_bot, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
    )

    navigate_warehouse = Node(
        package='warehouse_bot',
        executable='navigate_warehouse.py',
        name='navigate_warehouse',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        gzserver,
        robot_state_pub,
        spawn,
        nav2_bringup,
        navigate_warehouse,
    ])
