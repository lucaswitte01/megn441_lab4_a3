import os
from ament_index_python.packages import get_package_share_directory

import launch
# import launch_ros.actions
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Find the location of the package on your computer
    obcam_pkg_path = get_package_share_directory('orbbec_camera')

    depth_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(obcam_pkg_path, 'launch/dabai_dcw.launch.py')),
        launch_arguments={'camera_name': 'depth_cam',
                          'depth_registration': 'true',
                          'align_mode':'HW'}.items()
    )

    block_node = Node(
        package='block_detector',
        executable='block_detector',
        name='block_detector',
        output='screen'
    )

    rqt_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        output='screen'
    )

    return launch.LaunchDescription([
        depth_cam_launch,
        block_node,
        rqt_node
    ])