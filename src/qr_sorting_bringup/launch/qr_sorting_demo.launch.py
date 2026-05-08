from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    use_fake_qr = LaunchConfiguration('use_fake_qr')
    fake_qr_text = LaunchConfiguration('fake_qr_text')

    config_file = PathJoinSubstitution([
        FindPackageShare('qr_sorting_bringup'),
        'config',
        'qr_sorting.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera_head/color/image_raw',
            description='RGB camera image topic remapped into zbar_ros as image.',
        ),
        DeclareLaunchArgument(
            'use_fake_qr',
            default_value='false',
            description='Set true to publish a fake QR value through /barcode for testing.',
        ),
        DeclareLaunchArgument(
            'fake_qr_text',
            default_value='BIN_A',
            description='Fake QR text to publish when use_fake_qr is true.',
        ),

        Node(
            package='zbar_ros',
            executable='barcode_reader',
            name='barcode_reader',
            output='screen',
            remappings=[('image', image_topic)],
            arguments=['--ros-args', '--log-level', 'INFO'],
        ),

        Node(
            package='qr_sorting_logic',
            executable='qr_decision_node',
            name='qr_decision_node',
            output='screen',
            parameters=[config_file],
        ),

        Node(
            package='qr_sorting_mtc',
            executable='pick_scan_place_mtc',
            name='pick_scan_place_mtc',
            output='screen',
            parameters=[config_file],
        ),

        Node(
            package='qr_sorting_logic',
            executable='qr_test_injector',
            name='qr_test_injector',
            output='screen',
            parameters=[{'qr_text': fake_qr_text}],
            condition=IfCondition(use_fake_qr),
        ),
    ])
