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
    print(f"mundo_sugerido = '{mundo_sugerido}'")
    num_robots = int(LaunchConfiguration('num_robots').perform(context)) 

    # Lógica de mundo 
    if 'maze' in mundo_sugerido: 
        mundo, cenario_escolhido, base_x, base_y = 'tpf_arena.world', 'maze', -4.0, -4.0 
        goal_x, goal_y = 4.3, 4.3      
    elif 'simple' in mundo_sugerido: 
        mundo, cenario_escolhido, base_x, base_y = 'tpf_simple.world', 'simple', -4.0, -4.0
        goal_x, goal_y = 4.3, 4.3
    elif 'arena' in mundo_sugerido: 
        mundo, cenario_escolhido, base_x, base_y = 'tpf_arena02.world', 'arena', -3.2, -4.2 
        goal_x, goal_y = 4.3, 4.3
    elif 'empty' in mundo_sugerido:
        mundo, cenario_escolhido, base_x, base_y = 'empty.sdf', 'empty', 0.0, 0.0
        goal_x, goal_y = 4.3, 4.3
    else: 
        mundo, cenario_escolhido, base_x, base_y = 'tp2_simple.world', 'simple', -4.0, -4.0 
        goal_x, goal_y = 4.3, 4.3

    # Gazebo 
    world_path = 'empty.sdf' if cenario_escolhido == 'empty' else os.path.join(pkg_share, 'worlds', mundo)

    
    gazebo = IncludeLaunchDescription( 

        
PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ros_gz_sim'),
 'launch', 'gz_sim.launch.py')]), 
        launch_arguments={'gz_args': f'-r -v 4 {world_path}'}.items() 
    ) 

    nodes_list = []                     # lista vazia
    nodes_list.append(gazebo)          
    
    if cenario_escolhido != 'empty': 

        nodes_list.append(Node(package=package_name, 
executable='occupancy_grid', parameters=[{'scenario': 
cenario_escolhido}])) 

    # URDF 
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro') 
    robot_desc_xml = xacro.process_file(xacro_file).toxml() 

    # Criação dos Robôs (Padronizado para robot_{i})
    spawn_params = [] 
    for i in range(num_robots): 
        ns = f'robot_{i}' 
        gz_model_name = ns  # Nome do modelo no Gazebo igual ao namespace 

        spawn_x = str(base_x + (i % 3) * 0.6) 
        spawn_y = str(base_y + (i // 3) * 0.6)
        spawn_params.append(('spawn_x_{}'.format(i), spawn_x))
        spawn_params.append(('spawn_y_{}'.format(i), spawn_y))

        xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro') 
        # Passa o argumento robot_name para o xacro em tempo de execução
        robot_desc_xml = xacro.process_file(
            xacro_file, 
            mappings={'robot_name': ns}
        ).toxml() 

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
gz_model_name, '-x', spawn_x, '-y', spawn_y, '-z', '0.2','-allow_renaming', 'false'] 
        )) 

        # 4. PONTE DE COMUNICAÇÃO ISOLADA E ESPECÍFICA 
        # Bridge que funciona com Gazebo Harmonic (Sem erro de caminho) 
        bridge = Node( 
            package='ros_gz_bridge', 
            executable='parameter_bridge', 
            arguments=[ 
                # We put the namespace directly into the Gazebo topic definition
                f'/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist', 
                f'/{ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry', 
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' 
            ], 
            output='screen'
            # REMOVE remappings and namespace=ns from here!
        ) 
        nodes_list.append(bridge) 
          
# No launch, dentro da lista de parâmetros do nó Diff_SPH
    parameters=[{
        'use_sim_time': True,
        'num_robots': num_robots,
        'goal_x': goal_x,
        'goal_y': goal_y,
        **{f'spawn_x_{i}': float(base_x + (i % 3) * 0.6) for i in range(num_robots)},
        **{f'spawn_y_{i}': float(base_y + (i // 3) * 0.6) for i in range(num_robots)}
    }]
    nodes_list.append(Node(package=package_name, executable=modo, 
output='screen', parameters=parameters)) 
    return nodes_list 


def generate_launch_description(): 
    return LaunchDescription([ 
        DeclareLaunchArgument('modo', default_value='diff_sph'), 
        DeclareLaunchArgument('mundo', default_value='empty'), 
        DeclareLaunchArgument('num_robots', default_value='6'), 
        OpaqueFunction(function=launch_setup) 
    ]) 