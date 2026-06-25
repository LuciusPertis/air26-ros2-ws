import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import copy
from launch.substitutions import LaunchConfiguration
import sys
sys.path.append(str(os.path.join(get_package_share_directory('realsense2_camera'), 'launch')))
import rs_launch

json_path = os.path.join(get_package_share_directory('stretch_core'), 'config', 'HighAccuracyPreset.json')

# Starting with `realsense_ros` 4.55.1, the `.profile`` parameter is split by stream type.
# Here, we keep both the old and new parameters for backwards compatibility.
# https://github.com/IntelRealSense/realsense-ros/pull/3052
configurable_parameters = [{'name': 'camera_namespace1',             'default': '', 'description': 'namespace for camera'},
                           {'name': 'camera_name1',                  'default': 'camera', 'description': 'camera unique name'},
                           {'name': 'device_type1',                  'default': 'd435', 'description': 'camera unique name'},
                           {'name': 'json_file_path1',               'default': json_path, 'description': 'allows advanced configuration'},
                           {'name': 'depth_module.profile1',         'default': '424x240x15', 'description': 'depth module profile'},
                           {'name': 'depth_module.depth_profile1',   'default': '424x240x15', 'description': 'depth module profile'},
                           {'name': 'depth_module.infra_profile1',   'default': '424x240x15', 'description': 'depth module profile'},
                           {'name': 'enable_depth1',                 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'rgb_camera.profile1',           'default': '424x240x15', 'description': 'color image width'},
                           {'name': 'rgb_camera.color_profile1',     'default': '424x240x15', 'description': 'color image width'},
                           {'name': 'enable_color1',                 'default': 'true', 'description': 'enable color stream'},
                           {'name': 'enable_infra11',                'default': 'true', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra21',                'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'infra_rgb1',                    'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'enable_confidence1',            'default': 'false', 'description': 'enable depth stream'},
                           {'name': 'gyro_fps1',                     'default': '200', 'description': "''"},
                           {'name': 'accel_fps1',                    'default': '100', 'description': "''"},
                           {'name': 'enable_gyro1',                  'default': 'true', 'description': "''"},
                           {'name': 'enable_accel1',                 'default': 'true', 'description': "''"},
                           {'name': 'pointcloud.enable1',            'default': 'true', 'description': ''}, 
                           {'name': 'pointcloud.stream_filter1',     'default': '2', 'description': 'texture stream for pointcloud'},
                           {'name': 'pointcloud.stream_index_filter1','default': '0', 'description': 'texture stream index for pointcloud'},
                           {'name': 'enable_sync1',                  'default': 'true', 'description': "''"},
                           {'name': 'align_depth.enable1',           'default': 'true', 'description': "''"},
                           {'name': 'initial_reset1',                'default': 'true', 'description': "''"},
                           {'name': 'allow_no_texture_points1',      'default': 'true', 'description': "''"},
                           {'name': 'camera_namespace2',             'default': '', 'description': 'namespace for camera'},
                           {'name': 'camera_name2',                  'default': 'gripper_camera', 'description': 'camera unique name'},
                           {'name': 'device_type2',                  'default': 'd405', 'description': 'camera unique name'},
                           {'name': 'json_file_path2',               'default': json_path, 'description': 'allows advanced configuration'},
                           {'name': 'depth_module.profile2',         'default': '480x270x15', 'description': 'depth module profile'},
                           {'name': 'depth_module.depth_profile2',   'default': '480x270x15', 'description': 'depth module profile'},
                           {'name': 'depth_module.infra_profile2',   'default': '480x270x15', 'description': 'depth module profile'},
                           {'name': 'depth_module.enable_auto_exposure2', 'default': 'true', 'description': 'enable/disable auto exposure for depth image'},
                           {'name': 'enable_depth2',                 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'rgb_camera.profile2',           'default': '424x240x15', 'description': 'color image profile'},
                           {'name': 'depth_module.color_profile2',   'default': '424x240x15', 'description': 'color image profile'},
                           {'name': 'rgb_camera.enable_auto_exposure2', 'default': 'true', 'description': 'enable/disable auto exposure for color image'},
                           {'name': 'enable_color2',                 'default': 'true', 'description': 'enable color stream'},
                           {'name': 'enable_infra12',                'default': 'false', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra22',                'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'infra_rgb2',                    'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'enable_confidence2',            'default': 'false', 'description': 'enable depth stream'},
                           {'name': 'gyro_fps2',                     'default': '200', 'description': "''"},
                           {'name': 'accel_fps2',                    'default': '100', 'description': "''"},
                           {'name': 'enable_gyro2',                  'default': 'true', 'description': "''"},
                           {'name': 'enable_accel2',                 'default': 'true', 'description': "''"},
                           {'name': 'pointcloud.enable2',            'default': 'true', 'description': ''}, 
                           {'name': 'pointcloud.stream_filter2',     'default': '2', 'description': 'texture stream for pointcloud'},
                           {'name': 'pointcloud.stream_index_filter2','default': '0', 'description': 'texture stream index for pointcloud'},
                           {'name': 'enable_sync2',                  'default': 'true', 'description': "''"},
                           {'name': 'align_depth.enable2',           'default': 'true', 'description': "''"},
                           {'name': 'initial_reset2',                'default': 'false', 'description': "''"},
                           {'name': 'allow_no_texture_points2',      'default': 'true', 'description': "''"}, 
                          ]

def set_configurable_parameters(local_params):
    return dict([(param['original_name'], LaunchConfiguration(param['name'])) for param in local_params])

def duplicate_params(general_params, posix):
    local_params = copy.deepcopy(general_params)
    for param in local_params:
        param['original_name'] = param['name']
        param['name'] += posix
    return local_params

def generate_launch_description():
    params1 = duplicate_params(rs_launch.configurable_parameters, '1')
    params2 = duplicate_params(rs_launch.configurable_parameters, '2')

    d435i_accel_correction = Node(
          package='stretch_core',
          executable='d435i_accel_correction',
          output='screen',
     )
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(configurable_parameters) +
        rs_launch.declare_configurable_parameters(params1) +
        rs_launch.declare_configurable_parameters(params2) +
        [
        OpaqueFunction(function=rs_launch.launch_setup,
                       kwargs = {'params'           : set_configurable_parameters(params1),
                                 'param_name_suffix': '1'}),
        OpaqueFunction(function=rs_launch.launch_setup,
                       kwargs = {'params'           : set_configurable_parameters(params2),
                                 'param_name_suffix': '2'}),
    ]+[d435i_accel_correction])