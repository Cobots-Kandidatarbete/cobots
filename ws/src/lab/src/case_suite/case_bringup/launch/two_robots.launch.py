import os
import sys
import json
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    scenario = "two_robots_scenario"
    print("Generating launch for Robot 3")
    tf_scene_dir = FindPackageShare("case_scenes").find("case_scenes")
    bringup_dir = FindPackageShare("case_bringup").find("case_bringup")
    ur_setup_dir = FindPackageShare("ur_setup").find("ur_setup")
    ursg_setup_dir = FindPackageShare(
        "ur_script_generator").find("ur_script_generator")

    

    # r3_robot_parameters_path = os.path.join(
    #     ur_setup_dir, "robots", "case_r3", "general.json"
    # )

    # r4_robot_parameters_path = os.path.join(
    #     ur_setup_dir, "robots", "case_r4", "general.json"
    # )

    parameters = {
        "templates_path": os.path.join(ursg_setup_dir),
        "scenario_path": os.path.join(tf_scene_dir, "scenarios", scenario),
        "meshes_path": "/ros/ia_ros_meshes"
    }

    # with open(r3_robot_parameters_path) as jsonfile:
    #     r3_robot_parameters = json.load(jsonfile)

    r3_robot_parameters = {
        "name": "case_r3",
        "ur_type": "ur3e",
        "ip_address": "192.168.100.32",
        "prefix": "r3",
        "rtde_port": 30003
    }

    r4_robot_parameters = {
        "name": "case_r4",
        "ur_type": "ur3e",
        "ip_address": "192.168.100.42",
        "prefix": "r4",
        "rtde_port": 30003
    }

    #r3 description arguments
    r3_declared_arguments = []

    r3_declared_arguments.append(
        DeclareLaunchArgument(
            "r3_ur_type",
            default_value=r3_robot_parameters["ur_type"],
            description="Type/series of used UR robot.",
        )
    )

    r3_declared_arguments.append(
        DeclareLaunchArgument(
            "r3_safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    r3_declared_arguments.append(
        DeclareLaunchArgument(
            "r3_safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    r3_declared_arguments.append(
        DeclareLaunchArgument(
            "r3_safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    r3_declared_arguments.append(
        DeclareLaunchArgument(
            "r3_description_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    r3_declared_arguments.append(
        DeclareLaunchArgument(
            "r3_description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    r3_declared_arguments.append(
        DeclareLaunchArgument(
            "r3_prefix",
            default_value=r3_robot_parameters["prefix"] + "_",
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )

    r3_ur_type = LaunchConfiguration("r3_ur_type")
    r3_safety_limits = LaunchConfiguration("r3_safety_limits")
    r3_safety_pos_margin = LaunchConfiguration("r3_safety_pos_margin")
    r3_safety_k_position = LaunchConfiguration("r3_safety_k_position")
    r3_description_package = LaunchConfiguration("r3_description_package")
    r3_description_file = LaunchConfiguration("r3_description_file")
    r3_prefix = LaunchConfiguration("r3_prefix")

    r3_joint_limit_params = PathJoinSubstitution(
        [
            FindPackageShare("ur_setup"),
            "robots",
            r3_robot_parameters["name"],
            "joint_limits.yaml",
        ]
    )
    r3_kinematics_params = PathJoinSubstitution(
        [
            FindPackageShare("ur_setup"),
            "robots",
            r3_robot_parameters["name"],
            "default_kinematics.yaml",
        ]
    )
    r3_physical_params = PathJoinSubstitution(
        [
            FindPackageShare("ur_setup"),
            "robots",
            r3_robot_parameters["name"],
            "physical_parameters.yaml",
        ]
    )
    r3_visual_params = PathJoinSubstitution(
        [
            FindPackageShare("ur_setup"),
            "robots",
            r3_robot_parameters["name"],
            "visual_parameters.yaml",
        ]
    )
    r3_script_filename = PathJoinSubstitution(
        [FindPackageShare(r3_description_package), "resources",
         "ros_control.urscript"]
    )
    r3_input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare(r3_description_package), "resources",
         "rtde_input_recipe.txt"]
    )
    r3_output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare(r3_description_package), "resources",
         "rtde_output_recipe.txt"]
    )

    r3_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(r3_description_package),
                 "urdf", r3_description_file]
            ),
            " ",
            "joint_limit_params:=",
            r3_joint_limit_params,
            " ",
            "kinematics_params:=",
            r3_kinematics_params,
            " ",
            "physical_params:=",
            r3_physical_params,
            " ",
            "visual_params:=",
            r3_visual_params,
            " ",
            "safety_limits:=",
            r3_safety_limits,
            " ",
            "safety_pos_margin:=",
            r3_safety_pos_margin,
            " ",
            "safety_k_position:=",
            r3_safety_k_position,
            " ",
            "name:=",
            r3_ur_type,
            " ",
            "script_filename:=",
            r3_script_filename,
            " ",
            "input_recipe_filename:=",
            r3_input_recipe_filename,
            " ",
            "output_recipe_filename:=",
            r3_output_recipe_filename,
            " ",
            "prefix:=",
            r3_prefix,
        ]
    )
    r3_robot_description = {"robot_description": r3_robot_description_content}



    #r4 description arguments
    r4_declared_arguments = []

    r4_declared_arguments.append(
        DeclareLaunchArgument(
            "r4_ur_type",
            default_value=r4_robot_parameters["ur_type"],
            description="Type/series of used UR robot.",
        )
    )

    r4_declared_arguments.append(
        DeclareLaunchArgument(
            "r4_safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    r4_declared_arguments.append(
        DeclareLaunchArgument(
            "r4_safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    r4_declared_arguments.append(
        DeclareLaunchArgument(
            "r4_safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    r4_declared_arguments.append(
        DeclareLaunchArgument(
            "r4_description_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    r4_declared_arguments.append(
        DeclareLaunchArgument(
            "r4_description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    r4_declared_arguments.append(
        DeclareLaunchArgument(
            "r4_prefix",
            default_value=r4_robot_parameters["prefix"] + "_",
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )

    r4_ur_type = LaunchConfiguration("r4_ur_type")
    r4_safety_limits = LaunchConfiguration("r4_safety_limits")
    r4_safety_pos_margin = LaunchConfiguration("r4_safety_pos_margin")
    r4_safety_k_position = LaunchConfiguration("r4_safety_k_position")
    r4_description_package = LaunchConfiguration("r4_description_package")
    r4_description_file = LaunchConfiguration("r4_description_file")
    r4_prefix = LaunchConfiguration("r4_prefix")

    r4_joint_limit_params = PathJoinSubstitution(
        [
            FindPackageShare("ur_setup"),
            "robots",
            r4_robot_parameters["name"],
            "joint_limits.yaml",
        ]
    )
    r4_kinematics_params = PathJoinSubstitution(
        [
            FindPackageShare("ur_setup"),
            "robots",
            r4_robot_parameters["name"],
            "default_kinematics.yaml",
        ]
    )
    r4_physical_params = PathJoinSubstitution(
        [
            FindPackageShare("ur_setup"),
            "robots",
            r4_robot_parameters["name"],
            "physical_parameters.yaml",
        ]
    )
    r4_visual_params = PathJoinSubstitution(
        [
            FindPackageShare("ur_setup"),
            "robots",
            r4_robot_parameters["name"],
            "visual_parameters.yaml",
        ]
    )
    r4_script_filename = PathJoinSubstitution(
        [FindPackageShare(r4_description_package), "resources",
         "ros_control.urscript"]
    )
    r4_input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare(r4_description_package), "resources",
         "rtde_input_recipe.txt"]
    )
    r4_output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare(r4_description_package), "resources",
         "rtde_output_recipe.txt"]
    )

    r4_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(r4_description_package),
                 "urdf", r4_description_file]
            ),
            " ",
            "joint_limit_params:=",
            r4_joint_limit_params,
            " ",
            "kinematics_params:=",
            r4_kinematics_params,
            " ",
            "physical_params:=",
            r4_physical_params,
            " ",
            "visual_params:=",
            r4_visual_params,
            " ",
            "safety_limits:=",
            r4_safety_limits,
            " ",
            "safety_pos_margin:=",
            r4_safety_pos_margin,
            " ",
            "safety_k_position:=",
            r4_safety_k_position,
            " ",
            "name:=",
            r4_ur_type,
            " ",
            "script_filename:=",
            r4_script_filename,
            " ",
            "input_recipe_filename:=",
            r4_input_recipe_filename,
            " ",
            "output_recipe_filename:=",
            r4_output_recipe_filename,
            " ",
            "prefix:=",
            r4_prefix,
        ]
    )
    r4_robot_description = {"robot_description": r4_robot_description_content}

    rviz_config_file = os.path.join(bringup_dir, "config", "scenario_2.rviz")

    r3_driver_parameters = {
        "ur_address": r3_robot_parameters["ip_address"],
        "prefix": r3_robot_parameters["prefix"] + '_',
    }

    r4_driver_parameters = {
        "ur_address": r4_robot_parameters["ip_address"],
        "prefix": r4_robot_parameters["prefix"] + '_',
    }

    r3_ur_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=r3_robot_parameters["prefix"],
        output="screen",
        parameters=[r3_robot_description],
        #remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    r4_ur_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=r4_robot_parameters["prefix"],
        output="screen",
        parameters=[r4_robot_description],
        #remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    r3_ur_script_generator_node = Node(
        package="ur_script_generator",
        executable="ur_script_generator",
        namespace=r3_robot_parameters["prefix"],
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    r4_ur_script_generator_node = Node(
        package="ur_script_generator",
        executable="ur_script_generator",
        namespace=r4_robot_parameters["prefix"],
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    r3_ur_script_driver_node = Node(
        package="ur_script_driver",
        executable="ur_script_driver",
        namespace=r3_robot_parameters["prefix"],
        output="screen",
        parameters=[r3_driver_parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    r4_ur_script_driver_node = Node(
        package="ur_script_driver",
        executable="ur_script_driver",
        namespace=r4_robot_parameters["prefix"],
        output="screen",
        parameters=[r4_driver_parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    r3_ur_script_controller_node = Node(
        package="ur_script_controller",
        executable="ur_script_controller",
        namespace=r3_robot_parameters["prefix"],
        output="screen",
        parameters=[r3_driver_parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    r4_ur_script_controller_node = Node(
        package="ur_script_controller",
        executable="ur_script_controller",
        namespace=r4_robot_parameters["prefix"],
        output="screen",
        parameters=[r4_driver_parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace="",
        output="screen",
        arguments=["-d", rviz_config_file],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    tf_lookup_node = Node(
        package="tf_lookup",
        executable="tf_lookup",
        namespace="",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    tf_broadcast_node = Node(
        package="tf_broadcast",
        executable="tf_broadcast",
        namespace="",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    tf_sms_node = Node(
        package="tf_sms",
        executable="tf_sms",
        namespace="",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    viz_static_node = Node(
        package="viz_static",
        executable="viz_static",
        namespace="",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    viz_interactive_node = Node(
        package="viz_interactive",
        executable="viz_interactive",
        namespace="",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    viz_dynamic_node = Node(
        package="viz_dynamic",
        executable="viz_dynamic",
        namespace="",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace="",
        output="screen",
        arguments=["-d", rviz_config_file],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    control_gui_node = Node(
        package="gui_tools",
        executable="control_gui",
        namespace="",
        output="screen",
        arguments=["-d", rviz_config_file],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    query_gui_node = Node(
        package="gui_tools",
        executable="query_gui",
        namespace="",
        output="screen",
        arguments=["-d", rviz_config_file],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    marker_gui_node = Node(
        package="gui_tools",
        executable="marker_gui",
        namespace="",
        output="screen",
        arguments=["-d", rviz_config_file],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    aruco_node = Node(
        package="aruco_handler",
        executable="new_handler",
        namespace="",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    aruco_locker = Node(
        package="aruco_handler",
        executable="aruco_locker",
        namespace="",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    nodes_to_start = [
        r3_ur_robot_state_publisher_node,
        r3_ur_script_generator_node,
        r3_ur_script_driver_node,
        r3_ur_script_controller_node,
        r4_ur_robot_state_publisher_node,
        r4_ur_script_generator_node,
        r4_ur_script_driver_node,
        r4_ur_script_controller_node,
        tf_lookup_node,
        tf_broadcast_node,
        tf_sms_node,
        rviz_node,
        viz_static_node,
        viz_interactive_node,
        viz_dynamic_node,
        control_gui_node,
        query_gui_node,
        marker_gui_node,
        aruco_node,
        aruco_locker
    ]

    return LaunchDescription(r3_declared_arguments + r4_declared_arguments + nodes_to_start)


if __name__ == "__main__":
    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())
