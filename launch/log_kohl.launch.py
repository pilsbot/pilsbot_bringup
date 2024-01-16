import launch
import os

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag',
                 #'-b 34359738368', # ros2 does not seem to support the split size anymore
                 'record',
                 '/camera_info',
                 '/color/video/image',
                 'color/video/camera_info',
                 '/cmd_vel',
                 '/fix',
                 '/head_mcu',
                 '/heading',
                 '/hoverboard_api',
                 '/image_raw',
                 '/joint_states',
                 '/joy',
                 '/odom',
                 '/robot_description',
                 '/pilsbot_velocity_controller/cmd_vel',
                 '/pilsbot_velocity_controller/cmd_vel_out',
                 '/stereo/depth',
                 '/stereo/camera_info'
                 '/tf',
                 '/tf_static',
                 '/time_reference',
                 '/vel'],
            output='screen'
            ,cwd=
        )
    ])
