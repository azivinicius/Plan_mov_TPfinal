import os 
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription 
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction 
from launch.launch_description_sources import PythonLaunchDescriptionSource 
from launch.substitutions import LaunchConfiguration 
from launch_ros.actions import Node 
import xacro 

def launch_setup(context, *args, **kwargs): 
    package_name = 'SPH' 
    pkg_share = get_package_share_directory(package_name) 

    modo = LaunchConfiguration('modo').perform(context) 
    mundo_sugerido = LaunchConfiguration('mundo').perform(context) 
    num_robots = int(LaunchConfiguration('num_robots').perform(context)) 

    # Lógica de mundo 
    if 'maze' in mundo_sugerido: 
        mundo, cenario_escolhido, base_x, base_y = 'tp2_arena01.world', 'maze', -4.0, -4.0 
    elif 'complex' in mundo_sugerido: 
        mundo, cenario_escolhido, base_x, base_y = 'tp2_complex.world', 'complex', -1.0, -1.0 
    elif 'arena' in mundo_sugerido: 
        mundo, cenario_escolhido, base_x, base_y = 'tp2_arena02.world', 'arena', -3.2, -4.2 
    elif 'empty' in mundo_sugerido: 
        mundo, cenario_escolhido, base_x, base_y = 'empty.sdf', 'empty', 0.0, 0.0 
    else: 
        mundo, cenario_escolhido, base_x, base_y = 'tp2_complex.world', 'complex', -1.8, -1.8 

    # Gazebo 
    world_path = 'empty.sdf' if cenario_escolhido == 'empty' else os.path.join(pkg_share, 'worlds', mundo) 
    gazebo = IncludeLaunchDescription( 

         
PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ros_gz_sim'),
 'launch', 'gz_sim.launch.py')]), 
        launch_arguments={'gz_args': f'-r -v 4 {world_path}'}.items() 
    ) 

    nodes_list = [gazebo] 

    if cenario_escolhido != 'empty': 

        nodes_list.append(Node(package=package_name, 
executable='occupancy_grid', parameters=[{'scenario': 
cenario_escolhido}])) 

    # URDF 
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro') 
    robot_desc_xml = xacro.process_file(xacro_file).toxml() 

    # Criação dos Robôs (Padronizado para robot_{i}) 
    for i in range(num_robots): 
        ns = f'robot_{i}' 
        gz_model_name = ns  # Nome do modelo no Gazebo igual ao namespace 

        spawn_x = str(base_x + (i % 3) * 0.4) 
        spawn_y = str(base_y + (i // 3) * 0.4) 

        # RSP 
        nodes_list.append(Node( 
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            namespace=ns, 
            parameters=[{'robot_description': robot_desc_xml, 'use_sim_time': True, 'frame_prefix': f'{ns}/'}] 
        )) 

        # Spawner 
        nodes_list.append(Node( 
            package='ros_gz_sim', 
            executable='create', 

            arguments=['-topic', f'/{ns}/robot_description', '-name', 
gz_model_name, '-x', spawn_x, '-y', spawn_y, '-z', '0.2'] 
        )) 

        # 4. PONTE DE COMUNICAÇÃO ISOLADA E ESPECÍFICA 
        # Bridge que funciona com Gazebo Harmonic (Sem erro de caminho) 
        bridge = Node( 
            package='ros_gz_bridge', 
            executable='parameter_bridge', 
            arguments=[ 
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist', 
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry', 
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' 
            ], 
            remappings=[ 
                ('/cmd_vel', f'/{ns}/cmd_vel'), 
                ('/odom', f'/{ns}/odom'), 
            ], 
            output='screen', 
            namespace=ns # A mágica acontece aqui: ela joga o remapeamento DENTRO do namespace 
        ) 
        nodes_list.append(bridge) 
          

    nodes_list.append(Node(package=package_name, executable=modo, 
output='screen', parameters=[{'use_sim_time': True, 'num_robots': 
num_robots}])) 
    return nodes_list 


def generate_launch_description(): 
    return LaunchDescription([ 
        DeclareLaunchArgument('modo', default_value='diff_sph'), 
        DeclareLaunchArgument('mundo', default_value='empty'), 
        DeclareLaunchArgument('num_robots', default_value='6'), 
        OpaqueFunction(function=launch_setup) 
    ]) 