import launch_ros.actions
from launch import LaunchDescription


def generate_launch_description():

    # Nodes launching commands
    start_ultrasonic_hc_sr04_front_middle_cmd = launch_ros.actions.Node(
            package='ultrasonic_hc_sr04',
            executable='ultrasonic_hc_sr04',
            output='screen',
            parameters=[
                {'pigpio_host': 'localhost'},
                {'pigpio_port': '8888'},
                {'trigger_pin': 3},
                {'echo_pin': 2},
                {'topic': 'ultrasonic_distance_middle'},
                {'frequency': 20}, # tests showed that for now 47-50 Hz is the maximum possible frequency
                {'ultrasonic_sensor_frame': 'ultrasonic_sensor_front_middle_frame'}
            ]
            )
    
    start_ultrasonic_hc_sr04_front_left_cmd = launch_ros.actions.Node(
            package='ultrasonic_hc_sr04',
            executable='ultrasonic_hc_sr04',
            output='screen',
            parameters=[
                {'pigpio_host': 'localhost'},
                {'pigpio_port': '8888'},
                {'trigger_pin': 27},
                {'echo_pin': 17},
                {'topic': 'ultrasonic_distance_left'},
                {'frequency': 20}, # tests showed that for now 47-50 Hz is the maximum possible frequency
                {'ultrasonic_sensor_frame': 'ultrasonic_sensor_front_left_frame'}
            ]
            )
    
    start_ultrasonic_hc_sr04_front_right_cmd = launch_ros.actions.Node(
            package='ultrasonic_hc_sr04',
            executable='ultrasonic_hc_sr04',
            output='screen',
            parameters=[
                {'pigpio_host': 'localhost'},
                {'pigpio_port': '8888'},
                {'trigger_pin': 15},
                {'echo_pin': 14},
                {'topic': 'ultrasonic_distance_right'},
                {'frequency': 20}, # tests showed that for now 47-50 Hz is the maximum possible frequency
                {'ultrasonic_sensor_frame': 'ultrasonic_sensor_front_right_frame'}
            ]
            )

    ld = LaunchDescription()

    ld.add_action(start_ultrasonic_hc_sr04_front_middle_cmd)
    ld.add_action(start_ultrasonic_hc_sr04_front_left_cmd)
    ld.add_action(start_ultrasonic_hc_sr04_front_right_cmd)

    return ld