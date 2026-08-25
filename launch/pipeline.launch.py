r"""
Start every stage node of the merge pipeline plus its orchestrator.

The stage nodes are launched bare: pipeline_node pushes their parameters over
the parameter service before each Trigger call, so anything set on them here
would be overwritten. Tune the run through pipeline_node's own parameters -
the common ones are launch arguments below, the rest (yaw sweep / SAC-IA /
GICP knobs, timeouts) via `ros2 param set /pipeline_node <name> <value>`
before calling the service.

    ros2 launch my_point_reg pipeline.launch.py \
        source_pcd_path:=/abs/path/ground.pcd \
        target_pcd_path:=/abs/path/air.pcd \
        leaf_size:=0.1
    ros2 service call /run_pipeline std_srvs/srv/Trigger

Relative paths (including the default output_dir) resolve against the working
directory ros2 launch was started from.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

STAGE_NODES = [
    'voxel_node',
    'feature_node',
    'coarse_registration_node',
    'fine_registration_node',
    'merge_node',
    'dedup_node',
]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            'source_pcd_path',
            default_value='',
            description='Cloud to align onto the target (required).'),
        DeclareLaunchArgument(
            'target_pcd_path',
            default_value='',
            description='Cloud held fixed (required).'),
        DeclareLaunchArgument(
            'output_dir',
            default_value='output_test_file',
            description='Directory for all intermediate and merged clouds.'),
        DeclareLaunchArgument(
            'coarse_method',
            default_value='yaw_sweep',
            description=(
                'How the initial guess for GICP is found: yaw_sweep '
                'searches (x, y, yaw) exhaustively and needs no features, '
                'sac_ia matches FPFH descriptors and runs both feature '
                'stages first.')),
        DeclareLaunchArgument(
            'skip_coarse',
            default_value='false',
            description=(
                'Skip coarse registration entirely and start GICP from '
                'identity. Only safe when the pair is already near-aligned.')),
        DeclareLaunchArgument(
            'leaf_size',
            default_value='0.1',
            description='VoxelGrid leaf size applied to both clouds.'),
        DeclareLaunchArgument(
            'normal_radius',
            default_value='0.5',
            description='Normal estimation search radius.'),
        DeclareLaunchArgument(
            'fpfh_radius',
            default_value='1.0',
            description='FPFH feature search radius.'),
    ]

    stages = [
        Node(
            package='my_point_reg',
            executable=executable,
            name=executable,
            output='screen',
        )
        for executable in STAGE_NODES
    ]

    pipeline = Node(
        package='my_point_reg',
        executable='pipeline_node',
        name='pipeline_node',
        output='screen',
        parameters=[{
            # value_type is explicit: without it a substitution reaches the
            # node as a string and the double parameters are rejected.
            'source_pcd_path': ParameterValue(
                LaunchConfiguration('source_pcd_path'), value_type=str),
            'target_pcd_path': ParameterValue(
                LaunchConfiguration('target_pcd_path'), value_type=str),
            'output_dir': ParameterValue(
                LaunchConfiguration('output_dir'), value_type=str),
            'coarse_method': ParameterValue(
                LaunchConfiguration('coarse_method'), value_type=str),
            'skip_coarse': ParameterValue(
                LaunchConfiguration('skip_coarse'), value_type=bool),
            'leaf_size': ParameterValue(
                LaunchConfiguration('leaf_size'), value_type=float),
            'normal_radius': ParameterValue(
                LaunchConfiguration('normal_radius'), value_type=float),
            'fpfh_radius': ParameterValue(
                LaunchConfiguration('fpfh_radius'), value_type=float),
        }],
    )

    return LaunchDescription(arguments + stages + [pipeline])
