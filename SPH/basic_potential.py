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
# from SPH.potential import Potential
from SPH.potential import Potential

class Diff_SPH(Node):
    def __init__(self):
        super().__init__('Diff_SPH')
        
       # --- 1. SYSTEM PARAMETERS ---
        # Declare the parameter with a default fallback value (e.g., 6)
        self.declare_parameter('num_robots', 6)
        
        # Retrieve the value passed by the launch file
        self.num_robots = self.get_parameter('num_robots').value
        
        self.dt = 0.05
        
        # --- 2. MAP INFRASTRUCTURE ---
        self.map_recebido = False
        self.resolution = None
        self.origin_x = None
        self.origin_y = None
        self.grid_width = 0
        self.grid_height = 0
        self.grid = None
        
        # --- 3. ROBOT STATE AND CONTROL GAINS ---
        # Control gains for point-mass regulation
        self.d = 0.3  # Near-zero offset for punctual control
        self.kx = 1.0   # Increase proportional gain for faster point tracking
        self.ky = 1.0
        self.kv = 1.0
        self.kw = 1.0
        
        self.robot_x = [0.0] * self.num_robots
        self.robot_y = [0.0] * self.num_robots
        self.robot_theta = [0.0] * self.num_robots
        
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
        
        # --- 4. SPH INITIALIZATION ---
        self.particles = []
        n_side = int(math.ceil(math.sqrt(self.num_robots)))
        pos = np.linspace(-2.0, 2.0, n_side)
        
        idx = 0
        for i in range(n_side):
            for j in range(n_side):
                if idx < self.num_robots:
                    # particle.py expects: Particle(id, x, y, m)
                    p = Particle(id=str(idx), x=pos[i], y=pos[j], m=750.0)
                    self.particles.append(p)
                    idx += 1
                    
        # Initialize target potential (Assuming a 2D target at x=4, y=4)
        self.pot = Potential(xc=4.0, yc=4.0, R = 1.0)

        # --- 5. ROS 2 PUBLISHERS & SUBSCRIBERS ---
        self.pubs = []
        self.subs = []
        
        # Inicialize com listas vazias
        self.pubs = []
        self.subs = []
        
        # Crie os publishers/subscribers DEPOIS, mas garanta que o nó foi totalmente instanciado
        # Ou melhor, use um timer curto para verificar se os robôs estão presentes
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
            self.robot_x[index] = msg.pose.pose.position.x
            self.robot_y[index] = msg.pose.pose.position.y
            
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.robot_theta[index] = math.atan2(siny_cosp, cosy_cosp)
            print(f"Callback recebido para robô {index}: {msg.pose.pose.position.x}")
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
            p_obs = Particle(id=f"obs_{i}", x=real_x, y=real_y, m=1500.0)
            p_obs.rho = 5000.0 # Alta densidade fixa
            p_obs.fixed = True # Dica: você pode adicionar esse atributo na sua classe Particle
            
            self.particles.append(p_obs)

        self.map_recebido = True
        self.get_logger().info(f"Mapa processado. {len(self.particles) - self.num_robots} partículas de obstáculo adicionadas.")

    # --- MAIN CONTROL LOOP ---
    def control_loop(self):
        print
        print(f"Robô 0 pos atual: {self.robot_x[0]}")
        print(f"Robô 1 pos atual: {self.robot_x[1]}")
        print(f"Robô 2 pos atual: {self.robot_x[2]}")
        print(f"Robô 3 pos atual: {self.robot_x[3]}")
        # Registro de tempo 
        tempo_atual = self.get_clock().now().nanoseconds / 1e9
        self.hist_tempo.append(tempo_atual)

        # 1. ADVANCE SPH (Livre, sem forçar robô)
        for i in range(self.num_robots):
            self.particles[i].x = self.robot_x[i]
            self.particles[i].y = self.robot_y[i]
            self.particles[i].ext_force = self.pot.force(self.particles[i].x, self.particles[i].y)
            # self.particles[i].ext_force = [0.0, 0.0]
            f_x, f_y = self.pot.force(self.particles[i].x, self.particles[i].y)
            print(f"Robô {i} | Posição: ({self.particles[i].x:.2f}, {self.particles[i].y:.2f}) | Força: ({f_x:.4f}, {f_y:.4f})")

        # Atualização da física SPH
        particulas_dinamicas = [p for p in self.particles if not getattr(p, 'fixed', False)]
        new_rhos = [p.next_rho(self.particles) for p in particulas_dinamicas]
        for p, rho in zip(particulas_dinamicas, new_rhos): p.rho = rho
        
        new_Ps = [p.next_pressure() for p in particulas_dinamicas]
        for p, P in zip(particulas_dinamicas, new_Ps): p.P = P

        dvdts = [p.next_vel(self.particles) for p in particulas_dinamicas]
        es = [p.next_energy(self.particles) for p in particulas_dinamicas]
        
        for p, dvdt, e in zip(particulas_dinamicas, dvdts, es):
            p.update_particle(p.rho, p.P, dvdt, e, self.dt)

        # # 2. FECHAMENTO DE MALHA (Onde o robô persegue a partícula)
        # for i in range(self.num_robots):
        #     # Waypoint gerado pela partícula
        #     ref_x, ref_y = self.particles[i].x, self.particles[i].y
        #     rx, ry, rtheta = self.robot_x[i], self.robot_y[i], self.robot_theta[i]

        #     # Vetor erro
        #     erro_x = ref_x - rx
        #     erro_y = ref_y - ry
            
        #     dist = math.hypot(erro_x, erro_y)
        #     ang_erro = math.atan2(erro_y, erro_x) - rtheta
        #     ang_erro = (ang_erro + math.pi) % (2 * math.pi) - math.pi
            
        #     # Controle Proporcional
        #     v_cmd = self.kv * dist
        #     w_cmd = self.kw * ang_erro

        #     # --- HISTÓRICO MANTIDO ---
        #     self.hist_x_rob[i].append(rx)
        #     self.hist_y_rob[i].append(ry)
        #     # Salvando os dados da partícula (o waypoint) para seus gráficos
        #     self.hist_rho[i].append(self.particles[i].rho)
        #     self.hist_P[i].append(self.particles[i].P)
        #     self.hist_vel[i].append(self.particles[i].vel_norm()) 

        #     # Publicação
        #     twist = Twist()
        #     twist.linear.x = float(np.clip(v_cmd, -0.5, 0.5))
        #     twist.angular.z = float(np.clip(w_cmd, -1.0, 1.0))
        #     self.pubs[i].publish(twist)

# 2. COMPUTE AND PUBLISH ROBOT COMMANDS (Controle por aceleração/força)
        for i in range(self.num_robots):
            # Aceleração ditada pela física SPH + Força Externa
            ax = dvdts[i][0]
            ay = dvdts[i][1]

            # Integração para obter velocidade desejada (v_ref)
            # A classe Particle geralmente armazena a velocidade como .vx e .vy
            vx_ref = self.particles[i].vel[0] + ax * self.dt
            vy_ref = self.particles[i].vel[1] + ay * self.dt

            # Atualiza velocidade da partícula (próximo frame SPH)
            self.particles[i].vx, self.particles[i].vy = vx_ref, vy_ref

            # Transformar para v (linear) e w (angular)
            rx, ry, rtheta = self.robot_x[i], self.robot_y[i], self.robot_theta[i]

            v_ref = vx_ref * math.cos(rtheta) + vy_ref * math.sin(rtheta)
            w_ref = (1.0 / self.d) * (-vx_ref * math.sin(rtheta) + vy_ref * math.cos(rtheta))

            # --- HISTÓRICO MANTIDO ---
            self.hist_x_rob[i].append(rx)
            self.hist_y_rob[i].append(ry)
            self.hist_rho[i].append(self.particles[i].rho)
            self.hist_P[i].append(self.particles[i].P)
            self.hist_vel[i].append(self.particles[i].vel_norm())  # Certifique-se que este método existe na classe Particle

            # Publicação
            twist = Twist()
            twist.linear.x = float(np.clip(v_ref, -0.5, 0.5))
            twist.angular.z = float(np.clip(w_ref, -1.0, 1.0))
            self.pubs[i].publish(twist)

# --- NEW CODE: PLOT RESULTS METHOD ---
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

        # --- Graph 2: SPH Density (Rho) for Each Drone ---
        plt.subplot(2, 2, 2)
        for i in range(self.num_robots):
            plt.plot(t, self.hist_rho[i], linewidth=1.5, label=f'Robô {i}')
        plt.title('Densidade ($\\rho$) por Drone')
        plt.xlabel('Tempo (segundos)')
        plt.ylabel('Densidade')
        plt.grid(True)

        # --- Graph 3: SPH Pressure (P) for Each Drone ---
        plt.subplot(2, 2, 3)
        for i in range(self.num_robots):
            plt.plot(t, self.hist_P[i], linewidth=1.5, label=f'Robô {i}')
        plt.title('Pressão ($P$) por Drone')
        plt.xlabel('Tempo (segundos)')
        plt.ylabel('Pressão')
        plt.grid(True)

        # --- Graph 4: SPH Velocity Magnitude for Each Drone ---
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