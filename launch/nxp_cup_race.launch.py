from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetLaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from os import getlogin
from time import sleep

# Default path to world file
default_world_path = '/home/{:s}/git/nxp_gazebo/worlds/nxp_raceway.world'.format(str(getlogin()))

# Path to PX4 binary (in this case _sitl_rtps, could also be _sitl_default)
px4_path = '/home/{:s}/git/PX4-Autopilot/build/px4_sitl_rtps'.format(str(getlogin()))

# Path for PX4 binary storage
sitl_output_path = "/tmp/sitl_nxp_cupcar"

# Command to make storage folder
sitl_folder_cmd = ['mkdir -p \"{:s}\"'.format(sitl_output_path)]


# Command to export model and run PX4 binary
px4_cmd = '''export PX4_SIM_MODEL=\"nxp_cupcar\"; eval \"\"{:s}/bin/px4\" 
        -w {:s} \"{:s}/etc\" -s etc/init.d-posix/rcS\"; bash'''.format(
            px4_path, sitl_output_path, px4_path)

# Command to run micrortps agent
urtps_agent_cmd = '''eval \"micrortps_agent -t 
        {:s}\"; bash'''.format("UDP")

# Command to run micrortps client
urtps_client_cmd = '''eval \"{:s}/bin/px4-micrortps_client 
            start -t UDP\"'''.format(px4_path)

# Xterm command to name xterm window and run px4_cmd
xterm_px4_cmd = ['''xterm -hold -T \"PX4 NSH {:s}\" 
        -n \"PX4 NSH {:s}\" -e \'{:s}\''''.format(
            sitl_output_path, sitl_output_path,
            px4_cmd).replace("\n","").replace("    ","")]

# Xterm command to name xterm window and run urtps_agent_cmd
xterm_urtps_agent_cmd = ['''xterm -hold -T \"{:s}\" 
        -n \"{:s}\" -e \'{:s}\''''.format(
            "micrortps_agent", "micrortps_agent",
            urtps_agent_cmd).replace("\n",
                "").replace("    ","")]

# Xterm command to name xterm window and run urtps_client_cmd
xterm_urtps_client_cmd = ['''xterm -T \"{:s}\" 
        -n \"{:s}\" -e \'{:s}\''''.format(
            "micrortps_client", "micrortps_client",
            urtps_client_cmd).replace("\n",
                "").replace("    ","")]


def generate_launch_description():
    ld = LaunchDescription([
    	# World path argument
        DeclareLaunchArgument(
            'world_path', default_value= default_world_path,
            description='Provide full world file path and name'),
        LogInfo(msg=LaunchConfiguration('world_path')),
        ])

    
    # Get path to gazebo package
    gazebo_package_prefix = get_package_share_directory('gazebo_ros')

    # Launch gazebo servo with world file from world_path
    gazebo_server = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([gazebo_package_prefix,'/launch/gzserver.launch.py']),
                launch_arguments={'world': LaunchConfiguration('world_path')}.items(),
                )

    # Make storage command
    make_sitl_folder = ExecuteProcess(
        cmd=sitl_folder_cmd,
        name="make_sitl_folder",
        shell=True
    )
    
    ld.add_action(make_sitl_folder)

    # Run agent command
    micrortps_agent = ExecuteProcess(
        cmd=xterm_urtps_agent_cmd,
        name="xterm_urtps_agent",
        shell=True
    )
    
    ld.add_action(micrortps_agent)

    # Run PX4 binary
    px4_posix = ExecuteProcess(
        cmd=xterm_px4_cmd,
        name="xterm_px4_nsh",
        shell=True
    )
    
    ld.add_action(px4_posix)


    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    # Launch spawn model with nxp_cupcar
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'nxp_cupcar', '-database', 'nxp_cupcar'],
                        name="spawn_nxp_cupcar", output='screen')

    # Launch gazebo client
    gazebo_client = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([gazebo_package_prefix,'/launch/gzclient.launch.py']))

    # Run nxp_cup_vision node
    nxp_track_vision = Node(package="nxp_cup_vision", executable="nxp_track_vision",
                            name="nxp_track_vision", output='screen',
                            parameters=[{"pyramid_down": 2},
                                        {"debug": True}])

    # Run client command
    micrortps_client = ExecuteProcess(
        cmd=xterm_urtps_client_cmd,
        name="xterm_urtps_client",
        shell=True
    )

    ld.add_action(gazebo_server)
    ld.add_action(spawn_entity)
    ld.add_action(gazebo_client)
    LogInfo(msg="\nWaiting to launch Track Vision...\n")
    sleep(5)
    ld.add_action(nxp_track_vision)
    LogInfo(msg="\nTrack Vision Launched!\n")
    LogInfo(msg="\nWaiting to launch microRTPS Client...\n")
    sleep(2)
    ld.add_action(micrortps_client)
    LogInfo(msg="\nmicroRTPS Client Launched!\n")

    return ld
