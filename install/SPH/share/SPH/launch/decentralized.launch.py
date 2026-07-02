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

    # Lógica de mundo (mantida igual)
    if 'maze' in mundo_sugerido: 
        mundo, cenario_escolhido, base_x, base_y = 'tpf_arena.world', 'maze', -4.0, -4.0 
        goal_x, goal_y = 4.3, 4.3      
    elif 'simple' in mundo_sugerido: 
        mundo, cenario_escolhido, base_x, base_y = 'tpf_simple.world', 'simple', -4.0, -4.0
        goal_x, goal_y = 4.3, 4.3
    else: 
        mundo, cenario_escolhido, base_x, base_y = 'empty.sdf', 'empty', 0.0, 0.0
        goal_x, goal_y = 4.3, 4.3

    world_path = 'empty.sdf' if cenario_escolhido == 'empty' else os.path.join(pkg_share, 'worlds', mundo)

    gazebo = IncludeLaunchDescription( 
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]), 
        launch_arguments={'gz_args': f'-r -v 4 {world_path}'}.items() 
    ) 

    nodes_list = [gazebo]          
    
    if cenario_escolhido != 'empty': 
        nodes_list.append(Node(package=package_name, executable='occupancy_grid', parameters=[{'scenario': cenario_escolhido}])) 

    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro') 
    robot_desc_xml = xacro.process_file(xacro_file).toxml() 

    # --- CRIAÇÃO DESCENTRALIZADA ---
    for i in range(num_robots): 
        ns = f'robot_{i}' 
        spawn_x = str(base_x + (i % 3) * 0.4) 
        spawn_y = str(base_y + (i // 3) * 0.4) 

        # 1. RSP
        nodes_list.append(Node( 
            package='robot_state_publisher', executable='robot_state_publisher', 
            namespace=ns, 
            parameters=[{'robot_description': robot_desc_xml, 'use_sim_time': True, 'frame_prefix': f'{ns}/'}] 
        )) 

        # 2. Spawner
        nodes_list.append(Node( 
            package='ros_gz_sim', executable='create', 
            arguments=['-topic', f'/{ns}/robot_description', '-name', ns, '-x', spawn_x, '-y', spawn_y, '-z', '0.2'] 
        )) 

        # 3. Bridge
        nodes_list.append(Node( 
            package='ros_gz_bridge', executable='parameter_bridge', 
            arguments=['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist', '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'], 
            remappings=[('/cmd_vel', f'/{ns}/cmd_vel'), ('/odom', f'/{ns}/odom')], 
            namespace=ns 
        )) 

        # 4. Nó de Controle INDIVIDUAL (A descentralização acontece aqui)
        nodes_list.append(Node( 
            package=package_name, 
            executable=modo, 
            namespace=ns, # O nó roda isolado com seu próprio namespace
            output='screen', 
            parameters=[{
                'use_sim_time': True, 
                'num_robots': num_robots, 
                'goal_x': goal_x, 
                'goal_y': goal_y, 
                'id': i # Passando o ID único para cada nó
            }] 
        )) 

    return nodes_list 

def generate_launch_description(): 
    return LaunchDescription([ 
        DeclareLaunchArgument('modo', default_value='diff_sph'), 
        DeclareLaunchArgument('mundo', default_value='empty'), 
        DeclareLaunchArgument('num_robots', default_value='6'), 
        OpaqueFunction(function=launch_setup) 
    ])