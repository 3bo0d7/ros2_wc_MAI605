"""Launch the complete MAI605 QR sorting demo.

This bringup file starts the Panda MoveIt stack, simulated QR perception,
decision logic, RViz workcell visualization, zbar_ros, and the MoveIt Task
Constructor planner. It is intentionally parameterized so the same launch file
can test BIN_A, BIN_B, BIN_C, and UNKNOWN by changing the qr_text argument.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Launch arguments make the demo easy to test from the terminal:
    #   qr_text:=BIN_A/BIN_B/BIN_C/UNKNOWN
    #   launch_rviz:=true/false
    qr_text = LaunchConfiguration('qr_text')
    image_topic = LaunchConfiguration('image_topic')
    launch_rviz = LaunchConfiguration('launch_rviz')

    config_file = PathJoinSubstitution([
        FindPackageShare('qr_sorting_bringup'),
        'config',
        'qr_sorting.yaml',
    ])

    # The available robot in this environment is the Panda from MoveIt
    # resources, not MyCobot. MoveItConfigsBuilder loads robot_description,
    # SRDF, kinematics, planning pipelines, and controller parameters.
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_moveit_configs()

    # Fixed transforms keep the RViz workcell in a stable frame. The camera
    # frame matches the simulated scanner marker and QR image publisher.
    static_tf_world_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_panda_link0',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'panda_link0'],
    )

    static_tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='panda_to_qr_camera',
        output='screen',
        arguments=['0.35', '-0.25', '0.62', '0', '0', '0', 'panda_link0', 'qr_camera_frame'],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            {
                'rate': 30,
                # Start Panda near the MoveIt "ready" pose with an open hand so
                # the first MTC stages are visually consistent in RViz.
                'zeros': {
                    'panda_joint1': 0.0,
                    'panda_joint2': -0.785,
                    'panda_joint3': 0.0,
                    'panda_joint4': -2.356,
                    'panda_joint5': 0.0,
                    'panda_joint6': 1.571,
                    'panda_joint7': 0.785,
                    'panda_finger_joint1': 0.04,
                    'panda_finger_joint2': 0.04,
                },
            },
        ],
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[moveit_config.to_dict()],
        condition=IfCondition(launch_rviz),
    )

    sim_qr_camera = Node(
        package='qr_sorting_logic',
        executable='sim_qr_camera_node',
        name='sim_qr_camera_node',
        output='screen',
        parameters=[{'qr_text': qr_text}],
    )

    barcode_reader = Node(
        package='zbar_ros',
        executable='barcode_reader',
        name='barcode_reader',
        output='screen',
        # zbar_ros expects its input topic to be named "image". This remap keeps
        # the simulated camera topic namespaced under /qr_sort.
        remappings=[('image', image_topic)],
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    qr_decision_node = Node(
        package='qr_sorting_logic',
        executable='qr_decision_node',
        name='qr_decision_node',
        output='screen',
        parameters=[config_file],
    )

    scene_visualizer_node = Node(
        package='qr_sorting_logic',
        executable='scene_visualizer_node',
        name='scene_visualizer_node',
        output='screen',
        parameters=[config_file],
    )

    pick_scan_place_mtc = Node(
        package='qr_sorting_mtc',
        executable='pick_scan_place_mtc',
        name='pick_scan_place_mtc',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            config_file,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'qr_text',
            default_value='BIN_A',
            description='QR content to render into the simulated camera image.',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/qr_sort/sim_camera/image_raw',
            description='Image topic consumed by zbar_ros.',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz with Panda MoveIt parameters.',
        ),

        static_tf_world_to_base,
        static_tf_base_to_camera,

        robot_state_publisher,
        joint_state_publisher,
        move_group,

        # RViz starts after move_group has begun publishing parameters. This
        # reduces startup warnings and helps the Motion Planning display load.
        TimerAction(period=2.0, actions=[rviz]),

        sim_qr_camera,
        barcode_reader,
        qr_decision_node,
        scene_visualizer_node,

        # The MTC node starts last so robot_description, move_group, zbar_ros,
        # and the visualization/decision nodes are already available.
        TimerAction(period=6.0, actions=[pick_scan_place_mtc]),
    ])
