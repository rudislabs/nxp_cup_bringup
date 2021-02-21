from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetLaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from os import getlogin

world_path = '/home/{:s}/git/nxp_gazebo/worlds/nxp_raceway.world'.format(str(getlogin()))

px4_path = '/home/{:s}/git/PX4-Autopilot/build/px4_sitl_rtps'.format(str(getlogin()))

sitl_output_path = "/tmp/sitl_nxp_cupcar"

sitl_folder_cmd = ['mkdir -p \"{:s}\"'.format(sitl_output_path)]

px4_cmd = '''export PX4_SIM_MODEL=\"nxp_cupcar\"; eval \"\"{:s}/bin/px4\" 
        -w {:s} \"{:s}/etc\" -s etc/init.d-posix/rcS\"; bash'''.format(
            px4_path, sitl_output_path, px4_path)

mrtps_agent_cmd = '''eval \"micrortps_agent -t 
        {:s}\"; bash'''.format("UDP")

xterm_px4_cmd = ['''xterm -hold -T \"{:s}\" 
        -n \"{:s}\" -e \'{:s}\''''.format(
            sitl_output_path, sitl_output_path,
            px4_cmd).replace("\n","").replace("    ","")]

xterm_mrtps_agent_cmd = ['''xterm -hold -T \"{:s}\" 
        -n \"{:s}\" -e \'{:s}\''''.format(
            "micrortps_agent", "micrortps_agent",
            mrtps_agent_cmd).replace("\n",
                "").replace("    ","")]


def generate_launch_description():
    ld = LaunchDescription([

        DeclareLaunchArgument(
            'world', default_value='/home/{:s}/git/nxp_gazebo/worlds/nxp_raceway_octagon.world'.format(str(getlogin())),
            description='Specify world file name'),
        ])

    gazebo_package_prefix = get_package_share_directory('gazebo_ros')

    gazebo_server = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([gazebo_package_prefix,'/launch/gzserver.launch.py']),
                launch_arguments={'world': str(world_path)}.items(),
                )

    make_sitl_folder = ExecuteProcess(
        cmd=sitl_folder_cmd,
        shell=True
    )
    
    ld.add_action(make_sitl_folder)

    micrortps_agent = ExecuteProcess(
        cmd=xterm_mrtps_agent_cmd,
        shell=True
    )
    
    ld.add_action(micrortps_agent)


    px4_posix = ExecuteProcess(
        cmd=xterm_px4_cmd,
        shell=True
    )
    
    ld.add_action(px4_posix)


    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'nxp_cupcar', '-database', 'nxp_cupcar'],
                        output='screen')

    gazebo_client = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([gazebo_package_prefix,'/launch/gzclient.launch.py']))

    nxp_track_vision = Node(package="nxp_cup_vision", executable="nxp_track_vision",
                            parameters=[{"pyramid_down": 2},
                                        {"debug": True}])

    ld.add_action(gazebo_server)
    ld.add_action(spawn_entity)
    ld.add_action(gazebo_client)
    ld.add_action(nxp_track_vision)


    return ld
