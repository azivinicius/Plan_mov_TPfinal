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

    # Lógica de mundo: Atualizada com os novos limites (0 a 10) e posição do Goal (9, 9)
    if 'maze' in mundo_sugerido: 
        mundo, cenario_escolhido, base_x, base_y = 'tpf_arena.world', 'maze', 1.0, 1.0 
        goal_x, goal_y = 9.0, 9.0      
    elif 'simple' in mundo_sugerido: 
        mundo, cenario_escolhido, base_x, base_y = 'tpf_simple.world', 'simple', 1.0, 1.0
        goal_x, goal_y = 9.0, 9.0
    elif 'arena' in mundo_sugerido: 
        mundo, cenario_escolhido, base_x, base_y = 'tpf_arena02.world', 'arena', 1.0, 1.0 
        goal_x, goal_y = 9.0, 9.0
    elif 'empty' in mundo_sugerido:
        mundo, cenario_escolhido, base_x, base_y = 'empty.sdf', 'empty', 0.0, 0.0
        goal_x, goal_y = 9.0, 9.0
    else: 
        mundo, cenario_escolhido, base_x, base_y = 'tpf_simple.world', 'simple', 1.0, 1.0 
        goal_x, goal_y = 9.0, 9.0

    # Inicialização do Gazebo 
    world_path = 'empty.sdf' if cenario_escolhido == 'empty' else os.path.join(pkg_share, 'worlds', mundo)

    gazebo = IncludeLaunchDescription( 
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]), 
        launch_arguments={'gz_args': f'-v 4 {world_path}'}.items() 
    ) 

    nodes_list = []
    nodes_list.append(gazebo)          
    
    # O bloco do occupancy_grid foi totalmente removido daqui.

    # Criação dos Robôs
    for i in range(num_robots): 
        ns = f'robot_{i}' 
        gz_model_name = ns  

        spawn_x = str(base_x + (i % 3) * 0.6) 
        spawn_y = str(base_y + (i // 3) * 0.6)

        xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro') 
        
        # Processamento do URDF passando o namespace para evitar conflitos de TF
        robot_desc_xml = xacro.process_file(
            xacro_file, 
            mappings={'robot_name': ns}
        ).toxml() 

        # Robot State Publisher 
        nodes_list.append(Node( 
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            namespace=ns, 
            parameters=[{'robot_description': robot_desc_xml, 'use_sim_time': True, 'frame_prefix': f'{ns}/'}] 
        )) 

        # Spawner do Gazebo 
        nodes_list.append(Node( 
            package='ros_gz_sim', 
            executable='create', 
            arguments=['-topic', f'/{ns}/robot_description', '-name', gz_model_name, '-x', spawn_x, '-y', spawn_y, '-z', '0.2','-allow_renaming', 'false'] 
        )) 

        # Ponte de Comunicação (Bridge) com tópicos explícitos
        bridge = Node( 
            package='ros_gz_bridge', 
            executable='parameter_bridge', 
            arguments=[ 
                f'/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist', 
                f'/{ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry', 
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' 
            ], 
            output='screen'
        ) 
        nodes_list.append(bridge) 
          
    # Parâmetros unificados para o nó de controle principal (SPH)
    parameters = [{
        'scenario': cenario_escolhido,
        'use_sim_time': True,
        'num_robots': num_robots,
        'goal_x': goal_x,
        'goal_y': goal_y,
        **{f'spawn_x_{i}': float(base_x + (i % 3) * 0.6) for i in range(num_robots)},
        **{f'spawn_y_{i}': float(base_y + (i // 3) * 0.6) for i in range(num_robots)}
    }]
    
    # Nó de controle principal
    nodes_list.append(Node(
        package=package_name, 
        executable=modo, 
        name='Diff_SPH',      
        output='screen', 
        parameters=parameters
    ))
    
    return nodes_list 


def generate_launch_description(): 
    return LaunchDescription([ 
        DeclareLaunchArgument('modo', default_value='basic'), 
        DeclareLaunchArgument('mundo', default_value='empty'), 
        DeclareLaunchArgument('num_robots', default_value='6'), 
        OpaqueFunction(function=launch_setup) 
    ])