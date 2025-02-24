import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='sine_wave_package',
            executable='sine_wave_publisher',
            name='sine_wave_publisher',
            output='screen',
            parameters=['config/sine_wave_params.yaml']
        ),
        launch_ros.actions.Node(
            package='sine_wave_package',
            executable='sine_wave_subscriber',
            name='sine_wave_subscriber',
            output='screen'
        ),
    ])
