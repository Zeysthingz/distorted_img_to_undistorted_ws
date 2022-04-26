

from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python import get_package_share_directory
import os

# package path
image_rectifier_node_prefix = get_package_share_directory('img_rectifier')
image_rectifier_node_prefix_param_file = os.path.join(image_rectifier_node_prefix,
                                                  'params/params.yaml')
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='img_rectifier',
            executable='image_rectifier_node_exe',
            # Namespadce e bakarak aynı ıkı npodu farklı ısımlerle calıstırabılırsın
            # namespace='deneme',
            # parametreyi direkt yaml filedan okur.
            parameters=[image_rectifier_node_prefix_param_file],
            # to see c++ codes
            output='screen'


        )
    ])
