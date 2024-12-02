import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_block(i, model_name):
    model_path = os.path.join(get_package_share_directory('project_clearpath'), 'models')

    if model_name == "trash_block":
        x_pos = -1.4
        y_pos = 0.8
    elif model_name == "beer":
        x_pos = -1.8
        y_pos = 1.3
    elif model_name == "coke_can":
        x_pos = -2.2
        y_pos = 1.6
    elif model_name == "cricket_ball":
        x_pos = -5.0
        y_pos = 0.5
    else:
        x_pos = 1.0
        y_pos = 1.0

    return Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', f'{model_name}_{i}',
            '-file', os.path.join(model_path, model_name, 'model.sdf'),
            '-x', str(x_pos),
            '-y', str(y_pos),
            '-z', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )


def generate_launch_description():
    objects_to_spawn = ['trash_block', 'beer', 'coke_can', 'wooden_peg', 'cricket_ball']
    blocks = []

    for i, model_name in enumerate(objects_to_spawn):
        blocks.append(generate_block(i, model_name))

    return LaunchDescription(blocks)
