from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # urdf = "/home/jamesearljone/ros2_ws/src/real-world-robotics/ros/src/rwr_system/src/viz/models/URDF_Simulation/model_V5.urdf"
    urdf = os.path.join(
        get_package_share_directory('viz'),
        "models",
        "URDF_Simulation",
        "model_V6.urdf")
    # urdf = os.path.join(
    # get_package_share_directory('viz'),
    # "models",
    # "faive_hand_p4",
    # "urdf",
    # "p4.urdf")
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
        
    return LaunchDescription(
        [
            
            # Node(
            #     package="ingress",
            #     executable="mediapipe_node.py",
            #     name="mediapipe_node",
            #     output="log",
            # ),
            
            Node(
                package="ingress",
                executable="rokoko_node.py",
                name="rokoko_node",
                output="log",
                parameters=[
                    {"rokoko_tracker/ip": "0.0.0.0"},
                    {"rokoko_tracker/port": 14043},
                    {"rokoko_tracker/use_coil": True}
                ],
            ),

            # RETARGET NODE
            Node(
                package="retargeter",
                executable="retargeter_node.py",
                name="retargeter_node",
                output="screen",
                # COMMENT OR UNCOMMENT THE FOLLOWING LINES TO SWITCH BETWEEN MJCF AND URDF, JUST ONE OF THEM SHOULD BE ACTIVE TODO: Make this a parameter
                parameters=[
                    {
                        "retarget/urdf_filepath": os.path.join(
                            get_package_share_directory('viz'),
                            "models",
                            "URDF_Simulation",
                            "model_V6.urdf",
                        ),
                        "retarget/hand_scheme": os.path.join(
                            get_package_share_directory("viz"),
                            "models",
                            "URDF_Simulation",
                            "scheme_hand.yaml",
                        ),
                        "retarget/mano_adjustments": os.path.join(
                            get_package_share_directory("experiments"),
                            "cfgs",
                            "retargeter_adjustment_hand.yaml"
                        ),
                        "retarget/retargeter_cfg": os.path.join(
                            get_package_share_directory("experiments"),
                            "cfgs",
                            "retargeter_cfgs_hand.yaml"
                        ),
                    },
                    {"debug": True},
                ],
            ),
            
            # VISUALIZATION NODE
            Node(
                package="viz",
                executable="visualize_joints.py",
                name="visualize_joints",
                parameters=[
                    {
                        # "scheme_path": os.path.join(
                        #     get_package_share_directory("viz"),
                        #     "models",
                        #     "faive_hand_p4",
                        #     "scheme_p4.yaml",
                        # )
                        "scheme_path": os.path.join(
                            get_package_share_directory('viz'),
                            "models",
                            "URDF_Simulation",
                            "scheme_hand.yaml")
                    }
                ],
                output="screen",
            ),
            
                        
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_desc,}],
                arguments=[urdf]),
            
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen', 
                arguments=['-d', os.path.join(get_package_share_directory('viz'), 'rviz', 'retarget_config.rviz')],
                ),

        ]
    )
