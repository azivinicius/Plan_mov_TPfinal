import os 
import math 
import random 
import numpy as np 
from scipy.ndimage import label 
from collections import deque 

import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Twist, PoseStamped 
from nav_msgs.msg import Odometry, Path, OccupancyGrid 

# Import your custom modules 
from SPH.particle import Particle 
from SPH.potential2 import Potential 

class Diff_SPH(Node): 
    def __init__(self): 
        super().__init__('Diff_SPH') 
          
        # --- 1. SYSTEM PARAMETERS --- 
        # Declare the parameter with a default fallback value (e.g., 6) 
        self.declare_parameter('num_robots', 6) 
          
        # Retrieve the value passed by the launch file 
        self.num_robots = self.get_parameter('num_robots').value
          
        self.dt = 0.02
          
        # --- 2. MAP INFRASTRUCTURE --- 
        self.map_recebido = False 
        self.resolution = None 
        self.origin_x = None 
        self.origin_y = None 
        self.grid_width = 0 
        self.grid_height = 0 
        self.grid = None 
          
        # --- 3. ROBOT STATE AND CONTROL GAINS --- 

        self.d = 0.3 
        self.kx = 1.0  
        self.ky = 1.0 
        self.kv = 1.0 
        self.kw = 1.0 
          
        self.robot_x = [0.0] * self.num_robots 
        self.robot_y = [0.0] * self.num_robots 
        self.robot_theta = [0.0] * self.num_robots 
        
        self.declare_parameter('goal_x', 0.0)   # valor padrão se não vier do launch
        self.declare_parameter('goal_y', 0.0)
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.get_logger().info(f'Goal definido via launch: ({self.goal_x:.2f}, {self.goal_y:.2f})')

        # Moving average buffers 
        self.buffer_size_v = 5 
        self.buffer_size_w = 5 
        self.buffer_v = [deque([0.0] * self.buffer_size_v, maxlen=self.buffer_size_v) for _ in range(self.num_robots)] 
        self.buffer_w = [deque([0.0] * self.buffer_size_w, maxlen=self.buffer_size_w) for _ in range(self.num_robots)] 

        self.hist_tempo = [] 
        self.hist_x_rob = [[] for _ in range(self.num_robots)] 
        self.hist_y_rob = [[] for _ in range(self.num_robots)] 
        self.hist_rho = [[] for _ in range(self.num_robots)] 
        self.hist_P = [[] for _ in range(self.num_robots)] 
        self.hist_vel = [[] for _ in range(self.num_robots)] 

        self.hist_x_sph = [[] for _ in range(self.num_robots)]
        self.hist_y_sph = [[] for _ in range(self.num_robots)]
          
                # --- 4. SPH INITIALIZATION --- 
        self.particles = []
        self.odom_recebida = [False] * self.num_robots
        
        self.spawn_x = []
        self.spawn_y = []

        # Corrija o loop: declare e leia dentro dele
        for i in range(self.num_robots):
            self.declare_parameter(f'spawn_x_{i}', 0.0)
            self.declare_parameter(f'spawn_y_{i}', 0.0)
            
            x = float(self.get_parameter(f'spawn_x_{i}').value)
            y = float(self.get_parameter(f'spawn_y_{i}').value)
            
            self.spawn_x.append(x)
            self.spawn_y.append(y)
            
            # Cria a partícula com a posição de spawn
            p = Particle(id=str(i), x=x, y=y, m=1000.0)
            self.particles.append(p)
            
            self.get_logger().info(f"Partícula {i} criada em ({x:.3f}, {y:.3f})")

        # Initialize target potential (Assuming a 2D target at x=4, y=4) 
        # self.pot = Potential(xc=0.5, yc=0.05, R= 0.05) 
        self.pot = Potential(xc=self.goal_x, yc=self.goal_y)
        # --- 5. ROS 2 PUBLISHERS & SUBSCRIBERS --- 
        self.pubs = [] 
        self.subs = [] 

        self.create_timer(1.0, self.setup_publishers) 

    def setup_publishers(self):
        if len(self.pubs) == self.num_robots: return # Já criou
        
        for i in range(self.num_robots):
            # Garante que os tópicos existam ou espera
            pub = self.create_publisher(Twist, f'/robot_{i}/cmd_vel', 10)
            self.pubs.append(pub)
            
            sub = self.create_subscription(Odometry, f'/robot_{i}/odom', self.create_odom_callback(i), 10)
            self.subs.append(sub)
        self.get_logger().info("Publishers e Subscribers criados.")
        self.subscription_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10) 
        self.timer = self.create_timer(self.dt, self.control_loop) 

        self.get_logger().info(f"Diff_SPH Node Started controlling {self.num_robots} robots.") 

    # --- CALLBACKS --- 
    def create_odom_callback(self, index): 
        """Generates a dedicated odometry callback for a specific robot index.""" 
        def odom_callback(msg: Odometry): 
            # self.robot_x[index] = msg.pose.pose.position.x 
            # self.robot_y[index] = msg.pose.pose.position.y 
            self.robot_x[index] = self.spawn_x[index] + msg.pose.pose.position.x
            self.robot_y[index] = self.spawn_y[index] + msg.pose.pose.position.y
            q = msg.pose.pose.orientation 
            siny_cosp = 2 * (q.w * q.z + q.x * q.y) 
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z) 
            self.robot_theta[index] = math.atan2(siny_cosp, cosy_cosp)
            self.odom_recebida[index] = True
        return odom_callback 

    def map_callback(self, msg: OccupancyGrid): 
        if self.map_recebido: 
            return 

        self.get_logger().info("Map received! Processing metadata and obstacle particles...") 
        self.resolution = msg.info.resolution 
        self.origin_x = msg.info.origin.position.x 
        self.origin_y = msg.info.origin.position.y 
        self.grid_width = msg.info.width 
        self.grid_height = msg.info.height 

        # Processamento do grid 
        raw_grid = np.array(msg.data, dtype=np.int8).reshape((self.grid_height, self.grid_width)) 
        self.grid = np.zeros((self.grid_height, self.grid_width), dtype=int) 
          
        # Identifica obstáculos (raw_grid > 50 ou desconhecido -1) 
        obstacle_mask = (raw_grid > 50) | (raw_grid == -1) 
        self.grid[obstacle_mask] = 1  
          
        # --- NOVO: Inicializar partículas de alta densidade para cada obstáculo --- 
        # Coordenadas onde existe obstáculo 
        obs_y_indices, obs_x_indices = np.where(self.grid == 1) 
          
        # Filtra para não adicionar partículas demais (amostragem, se necessário) 
        # Aqui adicionamos uma a cada N obstáculos para não sobrecarregar o SPH 
        stride = 5   
        for i in range(0, len(obs_x_indices), stride): 
            x_idx = obs_x_indices[i] 
            y_idx = obs_y_indices[i] 
              
            # Converte de índice do grid para coordenadas reais 
            real_x = self.origin_x + (x_idx * self.resolution) 
            real_y = self.origin_y + (y_idx * self.resolution) 
              
            # Cria partícula obstáculo (massa alta para alta densidade) 
            p_obs = Particle(id=f"obs_{i}", x=real_x, y=real_y, m=5000.0) 
            p_obs.rho = 5000.0 # Alta densidade fixa 
            p_obs.fixed = True # Dica: você pode adicionar esse atributo na sua classe Particle 
              
            self.particles.append(p_obs) 

        self.map_recebido = True 

        self.get_logger().info(f"Mapa processado. {len(self.particles) - self.num_robots} partículas de obstáculo adicionadas.") 

    # --- MAIN CONTROL LOOP --- 
    def control_loop(self):
        
        if len(self.particles) < self.num_robots:
            return
        if not all(self.odom_recebida):
            return
        # Registro de tempo  
        tempo_atual = self.get_clock().now().nanoseconds / 1e9 
        self.hist_tempo.append(tempo_atual) 

        # 1. ADVANCE SPH (Livre, sem forçar robô) 
        for i in range(self.num_robots): 
            # self.particles[i].x = self.robot_x[i] 
            # self.particles[i].y = self.robot_y[i] 
            self.particles[i].ext_force = self.pot.force(self.particles[i].x, self.particles[i].y) 
            # TESTE PARA COMPORTAMENTO SEM FORÇA
            # self.particles[i].ext_force = [0.0, 0.0] 
            
        # Atualização da física SPH 
        # particulas_dinamicas = [p for p in self.particles if not getattr(p, 'fixed', False)] 
        particulas_dinamicas = self.particles
        resultados = []
        for p in particulas_dinamicas:
            # Nota: certifique-se que suas funções retornem APENAS o valor, 
            # sem atribuir a self.rho, self.P, etc.
            n_rho = p.next_rho(self.particles)
            n_P = p.next_pressure()
            dvdt = p.next_vel(self.particles)
            e = p.next_energy(self.particles)
            resultados.append((n_rho, n_P, dvdt, e))
        
        for p, (n_rho, n_P, dvdt, e) in zip(particulas_dinamicas, resultados):
                p.update_particle(n_rho, n_P, dvdt, e, self.dt)

        # 2. COMPUTE AND PUBLISH ROBOT COMMANDS (Controle por aceleração/força) 
        for i in range(self.num_robots): 
            self.hist_x_sph[i].append(self.particles[i].x)
            self.hist_y_sph[i].append(self.particles[i].y)

            ex = self.particles[i].x - self.robot_x[i]
            ey = self.particles[i].y - self.robot_y[i]
            k = 5
            vx_ref = self.particles[i].vel[0] + k * ex
            vy_ref = self.particles[i].vel[1] + k * ey

            rtheta = self.robot_theta[i]
            v_ref = vx_ref * math.cos(rtheta) + vy_ref * math.sin(rtheta)
            w_ref = (1.0 / self.d) * (-vx_ref * math.sin(rtheta) + vy_ref * math.cos(rtheta))

            self.hist_x_rob[i].append(self.robot_x[i])
            self.hist_y_rob[i].append(self.robot_y[i])
            self.hist_rho[i].append(self.particles[i].rho)
            self.hist_P[i].append(self.particles[i].P)
            self.hist_vel[i].append(self.particles[i].vel_norm())

            # Impressão APENAS para este robô (sem loop interno)
            dist = math.hypot(ex, ey)
            print(f"Robô {i:2d}: partícula ({self.particles[i].x:7.3f}, {self.particles[i].y:7.3f}) | "
      f"robô     ({self.robot_x[i]:7.3f}, {self.robot_y[i]:7.3f}) | "
      f"erro ({ex:7.3f}, {ey:7.3f}) | dist {dist:7.3f} | "
      f"v_ref={v_ref:6.3f} w_ref={w_ref:6.3f}")
            print(f"Robô {i}: partícula ({self.particles[i].x:.3f}, {self.particles[i].y:.3f}) | "
                f"vel ({self.particles[i].vel[0]:.3f}, {self.particles[i].vel[1]:.3f}) | "
                f"robô ({self.robot_x[i]:.3f}, {self.robot_y[i]:.3f})")
            print(f"  -> v_ref={v_ref:.3f}, w_ref={w_ref:.3f}")

            # Publicação APENAS para este robô
            twist = Twist()
            twist.linear.x = float(np.clip(v_ref, -0.5, 0.5))
            twist.angular.z = float(np.clip(w_ref, -1.0, 1.0))
            self.pubs[i].publish(twist)


    def plot_results(self): 
        """Generates and displays historical performance charts for all drones.""" 
        if not self.hist_tempo: 
            self.get_logger().warn("No historical data collected to plot.") 
            return 

        import numpy as np 
        import matplotlib 
        matplotlib.use('TkAgg')  
        import matplotlib.pyplot as plt 

        # Normalize time array 
        t = np.array(self.hist_tempo) - self.hist_tempo[0] 

        plt.close('all') 
        plt.figure(figsize=(14, 10)) 

        # --- Graph 1: XY Trajectories over the Map --- 
        plt.subplot(2, 2, 1) 
        if self.map_recebido: 
            map_img = np.zeros((self.grid_height, self.grid_width, 3), dtype=np.uint8) 
            map_img[self.grid == 0] = [255, 255, 255] 
            map_img[self.grid == 1] = [0, 0, 0] 
            x_min = self.origin_x 
            x_max = self.origin_x + self.grid_width * self.resolution 
            y_min = self.origin_y 
            y_max = self.origin_y + self.grid_height * self.resolution 
            plt.imshow(map_img, origin='lower', extent=[x_min, x_max, y_min, y_max], alpha=0.7) 

        for i in range(self.num_robots): 
            plt.plot(self.hist_x_rob[i], self.hist_y_rob[i], label=f'Robô {i}') 
          
        plt.scatter(self.pot.xc, self.pot.yc, color='red', marker='X', s=150, label='Potencial Objetivo') 
        plt.title('Trajetória no Plano XY') 
        plt.xlabel('X (metros)') 
        plt.ylabel('Y (metros)') 
        plt.grid(True) 
        plt.axis('equal') 

        # --- Graph 2: SPH Density (Rho) --- 
        plt.subplot(2, 2, 2) 
        for i in range(self.num_robots): 
            plt.plot(t, self.hist_rho[i], linewidth=1.5, label=f'Robô {i}') 
        plt.title('Densidade ($\\rho$) por Drone') 
        plt.xlabel('Tempo (segundos)') 
        plt.ylabel('Densidade') 
        plt.grid(True) 

        # --- Graph 3: SPH Pressure (P) --- 
        plt.subplot(2, 2, 3) 
        for i in range(self.num_robots): 
            plt.plot(t, self.hist_P[i], linewidth=1.5, label=f'Robô {i}') 
        plt.title('Pressão ($P$) por Drone') 
        plt.xlabel('Tempo (segundos)') 
        plt.ylabel('Pressão') 
        plt.grid(True) 

        # --- Graph 4: SPH Velocity Magnitude --- 
        plt.subplot(2, 2, 4) 
        for i in range(self.num_robots): 
            plt.plot(t, self.hist_vel[i], linewidth=1.5, label=f'Robô {i}') 
        plt.title('Norma da Velocidade SPH por Drone') 
        plt.xlabel('Tempo (segundos)') 
        plt.ylabel('Velocidade (m/s)') 
        plt.grid(True) 

        plt.tight_layout() 
        self.get_logger().info("Exibindo gráficos de desempenho... Feche a janela para finalizar.") 
        plt.show(block=True) 

def main(args=None): 
    rclpy.init(args=args) 
    node = Diff_SPH() 
    try: 
        rclpy.spin(node) 
    except KeyboardInterrupt: 
        node.get_logger().info("Nó interrompido pelo usuário. Gerando gráficos...") 
    finally: 
        # 1. Plot the graphs FIRST before the node completely dies 
        try: 
            node.plot_results() 
        except Exception as e: 
            print(f"Erro ao gerar gráficos: {e}") 

        # 2. Try to safely stop the robots (ignore if ROS 2 context is already dead) 
        try: 
            if rclpy.ok(): 
                for pub in node.pubs: 
                    twist_parada = Twist() 
                    pub.publish(twist_parada) 
        except Exception: 
            pass  

        node.destroy_node() 
        if rclpy.ok(): 
            rclpy.shutdown() 

if __name__ == '__main__': 
    main() 