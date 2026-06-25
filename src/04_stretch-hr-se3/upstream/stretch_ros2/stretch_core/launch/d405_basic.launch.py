import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

json_path = os.path.join(get_package_share_directory('stretch_core'), 'config', 'HighAccuracyPreset.json')

# Starting with `realsense_ros` 4.55.1, the `.profile`` parameter is split by stream type.
# Here, we keep both the old and new parameters for backwards compatibility.
# https://github.com/IntelRealSense/realsense-ros/pull/3052
configurable_parameters = [
                           {'name': 'camera_namespace',             'default': '', 'description': 'namespace for camera'},
                           {'name': 'camera_name',                  'default': 'gripper_camera', 'description': 'camera unique name'},
                           {'name': 'device_type',                  'default': 'd405', 'description': 'camera unique name'},
                           {'name': 'json_file_path',               'default': json_path, 'description': 'allows advanced configuration'},
                           {'name': 'depth_module.profile',         'default': '480x270x15', 'description': 'depth module profile'},
                           {'name': 'depth_module.depth_profile',   'default': '480x270x15', 'description': 'depth module profile'},
                           {'name': 'depth_module.infra_profile',   'default': '480x270x15', 'description': 'depth module profile'},
                           {'name': 'depth_module.enable_auto_exposure', 'default': 'true', 'description': 'enable/disable auto exposure for depth image'},
                           {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'rgb_camera.profile',           'default': '424x240x15', 'description': 'color image profile'},
                           {'name': 'depth_module.color_profile',   'default': '424x240x15', 'description': 'color image profile'},
                           {'name': 'rgb_camera.enable_auto_exposure', 'default': 'true', 'description': 'enable/disable auto exposure for color image'},
                           {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
                           {'name': 'enable_infra1',                'default': 'false', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra2',                'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'infra_rgb',                    'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'enable_confidence',            'default': 'false', 'description': 'enable depth stream'},
                           {'name': 'gyro_fps',                     'default': '200', 'description': "''"},
                           {'name': 'accel_fps',                    'default': '100', 'description': "''"},
                           {'name': 'enable_gyro',                  'default': 'true', 'description': "''"},
                           {'name': 'enable_accel',                 'default': 'true', 'description': "''"},
                           {'name': 'pointcloud.enable',            'default': 'true', 'description': ''}, 
                           {'name': 'pointcloud.stream_filter',     'default': '2', 'description': 'texture stream for pointcloud'},
                           {'name': 'pointcloud.stream_index_filter','default': '0', 'description': 'texture stream index for pointcloud'},
                           {'name': 'enable_sync',                  'default': 'true', 'description': "''"},
                           {'name': 'align_depth.enable',           'default': 'true', 'description': "''"},
                           {'name': 'initial_reset',                'default': 'false', 'description': "''"},
                           {'name': 'allow_no_texture_points',      'default': 'true', 'description': "''"}, 
                          ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():
     realsense_launch = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
               get_package_share_directory('realsense2_camera'), 'launch'),
               '/rs_launch.py'])
          )

     return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
          realsense_launch,
          ])
