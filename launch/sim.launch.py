#!/usr/bin/env python3
"""TurtleBot 4 in Ignition Gazebo (Fortress), headless, on macOS / RoboStack.

Same pieces as turtlebot4_ignition_bringup/turtlebot4_ignition.launch.py, minus what
needs a GUI or is not available on macOS (Gazebo GUI plugins, turtlebot4_viz):

    Gazebo server (world) -> TurtleBot 4 (turtlebot4_description) spawned
    ros_gz bridges (/scan, OAK-D camera, Create 3 sensors), Create 3 nodes
    (diff-drive controller via ros2_control, motion_control on /cmd_vel)
    optional: SLAM Toolbox | AMCL localization | Nav2 (turtlebot4_navigation launch
    files, with this package's parameter files)  +  RViz2

    ros2 launch turtlebot4_bai04 sim.launch.py                       # Phần 1
    ros2 launch turtlebot4_bai04 sim.launch.py slam:=true            # Phần 2
    ros2 launch turtlebot4_bai04 sim.launch.py localization:=true     # Phần 3
    ros2 launch turtlebot4_bai04 sim.launch.py localization:=true nav2:=true   # Phần 4
"""
import os
import re
import sys

import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

ROBOT_NAME = "turtlebot4"

# Sensor update rates [Hz] used in simulation. The stock description renders twelve GPU
# lidars at 62 Hz (RPLIDAR + Create 3 cliff/IR sensors) and the OAK-D at 30 Hz, which runs
# Gazebo at ~0.15x real time on a laptop GPU. 10 Hz is what an RPLIDAR A1 actually gives.
SENSOR_RATES = {"rplidar": 10, "cliff_": 10, "ir_intensity_": 10, "rgbd_camera": 5}

# Create 3 proximity sensors (4 cliff + 7 IR, all GPU lidars). SLAM, AMCL and Nav2 do not
# use them; with the ogre renderer on macOS the downward cliff rays mostly miss the floor
# (false cliffs make motion_control back up and ignore /cmd_vel) and the IR ones read back
# 2048x2056 textures each. Dropped unless proximity_sensors:=true.
PROXIMITY_SENSORS = ("cliff_", "ir_intensity_")


def drop_sensors(urdf, prefixes):
    return re.sub(
        r'<sensor[^>]*name="(' + "|".join(prefixes) + r')[^"]*"[^>]*>.*?</sensor>', "", urdf, flags=re.S)


def throttle_sensors(urdf):
    def set_rate(match):
        name, body = match.group(1), match.group(2)
        for prefix, rate in SENSOR_RATES.items():
            if name.startswith(prefix):
                body = re.sub(r"<update_rate>[^<]*</update_rate>", f"<update_rate>{rate}</update_rate>", body, count=1)
        return match.group(0)[: match.start(2) - match.start(0)] + body + "</sensor>"
    return re.sub(r'<sensor[^>]*name="([^"]+)"[^>]*>(.*?)</sensor>', set_rate, urdf, flags=re.S)


def macos_library_path():
    """macOS strips DYLD_LIBRARY_PATH from `ros2 launch` (a /usr/bin/env script, SIP), so
    composable-node executables (the Create 3 nodes) cannot dlopen their component
    libraries by name. Rebuild it from AMENT_PREFIX_PATH for every process started here."""
    if sys.platform != "darwin":
        return []
    libs = [os.path.join(p, "lib") for p in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep) if p]
    return [SetEnvironmentVariable("DYLD_LIBRARY_PATH", os.pathsep.join(libs))]


def render_engine():
    """The ogre plugin Gazebo's sensors render with. On macOS the stock plugin returns
    every gpu_lidar ray at range_min; macos/build_ogre_plugin.sh builds a patched one into
    <workspace>/ext/render_plugins, used here when present (TB4_OGRE_PLUGIN overrides)."""
    workspace = os.path.dirname(os.path.dirname(get_package_prefix("turtlebot4_bai04")))
    plugin = os.environ.get(
        "TB4_OGRE_PLUGIN",
        os.path.join(workspace, "ext", "render_plugins", "libignition-rendering6-ogre.dylib"))
    return plugin if os.path.exists(plugin) else "ogre"


def robot_state_publisher(context):
    model = LaunchConfiguration("model").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context) == "true"
    urdf = xacro.process_file(
        os.path.join(get_package_share_directory("turtlebot4_description"), "urdf", model, "turtlebot4.urdf.xacro"),
        mappings={"gazebo": "ignition", "namespace": ""},
    ).toxml()
    # The Create 3 description loads Gazebo's Sensors system with render_engine "ogre".
    engine = render_engine()
    urdf = urdf.replace("<render_engine>ogre</render_engine>", f"<render_engine>{engine}</render_engine>")
    urdf = throttle_sensors(urdf)
    if LaunchConfiguration("proximity_sensors").perform(context) != "true":
        urdf = drop_sensors(urdf, PROXIMITY_SENSORS)
    return [
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="robot_state_publisher", output="log",
            parameters=[{"use_sim_time": use_sim_time, "robot_description": urdf}],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="joint_state_publisher", executable="joint_state_publisher",
            name="joint_state_publisher", output="log",
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
    ]


def generate_launch_description():
    pkg = get_package_share_directory("turtlebot4_bai04")
    pkg_tb4_desc = get_package_share_directory("turtlebot4_description")
    pkg_tb4_ign = get_package_share_directory("turtlebot4_ignition_bringup")
    pkg_tb4_nav = get_package_share_directory("turtlebot4_navigation")
    pkg_create_desc = get_package_share_directory("irobot_create_description")
    pkg_create_common = get_package_share_directory("irobot_create_common_bringup")
    pkg_create_ign = get_package_share_directory("irobot_create_ignition_bringup")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    world = LaunchConfiguration("world")
    model = LaunchConfiguration("model")
    use_sim_time = LaunchConfiguration("use_sim_time")
    x, y, yaw = LaunchConfiguration("x"), LaunchConfiguration("y"), LaunchConfiguration("yaw")

    args = [
        DeclareLaunchArgument("world", default_value="maze",
                              description="World name (<world>.sdf in this package or turtlebot4_ignition_bringup)"),
        DeclareLaunchArgument("model", default_value="lite", choices=["lite", "standard"]),
        # Read (undeclared) by some of the included Create 3 / TurtleBot 4 launch files.
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("proximity_sensors", default_value="false", choices=["true", "false"],
                              description="Simulate the Create 3 cliff/IR sensors (see PROXIMITY_SENSORS)"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("slam", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("localization", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("nav2", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("map", default_value=os.path.join(pkg, "maps", "map.yaml"),
                              description="Map for localization (map.yaml)"),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(pkg, "rviz", "turtlebot4.rviz")),
    ]

    # Gazebo finds the world here and resolves the robot's model:// mesh URIs from the
    # description packages' parent directories.
    resource_paths = [
        AppendEnvironmentVariable(name, path)
        for name in ("IGN_GAZEBO_RESOURCE_PATH", "GZ_SIM_RESOURCE_PATH")
        for path in (
            os.path.join(pkg, "worlds"),
            os.path.join(pkg_tb4_ign, "worlds"),
            os.path.dirname(pkg_tb4_desc),
            os.path.dirname(pkg_create_desc),
        )
    ]

    # Server only (-s): the Gazebo GUI does not run on macOS; RViz2 is the viewer.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-s -r -v 3 ", world, ".sdf"]}.items(),
    )

    clock_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="clock_bridge", output="log",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
    )

    robot = GroupAction([
        OpaqueFunction(function=robot_state_publisher),
        Node(
            package="ros_gz_sim", executable="create", output="log",
            arguments=["-name", ROBOT_NAME, "-x", x, "-y", y, "-z", "0.0", "-Y", yaw,
                       "-topic", "robot_description"],
        ),
        # /scan (RPLIDAR), OAK-D camera, Create 3 sensors, HMI (standard model)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_tb4_ign, "launch", "ros_ign_bridge.launch.py")),
            launch_arguments={"model": model, "robot_name": ROBOT_NAME, "dock_name": "standard_dock",
                              "namespace": "", "world": world}.items(),
        ),
        # Create 3 base: diff-drive controller (ros2_control), motion_control (/cmd_vel), hazards, ...
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_create_common, "launch", "create3_nodes.launch.py")),
            launch_arguments={"namespace": ""}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_create_ign, "launch", "create3_ignition_nodes.launch.py")),
            launch_arguments={"robot_name": ROBOT_NAME, "dock_name": "standard_dock"}.items(),
        ),
        # Gazebo names sensor frames <model>/<link>/<sensor>; tie them to the URDF frames.
        Node(
            package="tf2_ros", executable="static_transform_publisher", name="rplidar_stf", output="log",
            arguments=["0", "0", "0", "0", "0", "0", "rplidar_link", f"{ROBOT_NAME}/rplidar_link/rplidar"],
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher", name="camera_stf", output="log",
            arguments=["0", "0", "0", "1.5707", "-1.5707", "0", "oakd_rgb_camera_optical_frame",
                       f"{ROBOT_NAME}/oakd_rgb_camera_frame/rgbd_camera"],
        ),
    ])

    def tb4_nav(name, extra):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_tb4_nav, "launch", f"{name}.launch.py")),
            launch_arguments=dict({"namespace": "", "use_sim_time": use_sim_time}, **extra).items(),
            condition=IfCondition(LaunchConfiguration(name)),
        )

    # Started a little later, once the robot's TF and /scan exist.
    navigation = TimerAction(period=8.0, actions=[
        tb4_nav("slam", {"params": os.path.join(pkg, "config", "slam.yaml")}),
        tb4_nav("localization", {"map": LaunchConfiguration("map"),
                                 "params": os.path.join(pkg, "config", "localization.yaml")}),
        TimerAction(period=4.0, actions=[
            tb4_nav("nav2", {"params_file": os.path.join(pkg, "config", "nav2.yaml")}),
        ]),
    ])

    # Without SLAM or AMCL there is no map frame yet (Phần 1): view in odom instead.
    fixed_frame = PythonExpression([
        "'map' if 'true' in ('", LaunchConfiguration("slam"), "', '",
        LaunchConfiguration("localization"), "') else 'odom'"])
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="log",
        arguments=["-d", LaunchConfiguration("rviz_config"), "-f", fixed_frame],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        args + macos_library_path() + resource_paths + [gazebo, clock_bridge, robot, navigation, rviz])
