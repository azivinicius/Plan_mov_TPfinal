import os 
import math 
import random 
import numpy as np 
from scipy.ndimage import label 
from collections import deque 

import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Twist, PoseStamped 
from nav_msgs.msg import Odometry, Path
import matplotlib.pyplot as plt 

from SPH.particle import Particle 
from SPH.potential import Potential 

# =====================================================================
# FUNÇÕES AUXILIARES DE MAPEAMENTO (0 A 10 METROS)
# =====================================================================
ORIGIN = -1.0
GRID_SIZE = 120   
RESOLUTION = 0.1  

def world_to_grid(x, y):
    col = int(round((x - ORIGIN) / RESOLUTION))
    row = int(round((y - ORIGIN) / RESOLUTION))
    return row, col

def draw_wall(grid, x, y, width_x, width_y):
    row_center, col_center = world_to_grid(x, y)
    w_cells = int(round((width_x / RESOLUTION) / 2))
    h_cells = int(round((width_y / RESOLUTION) / 2))
    
    r_min = max(0, row_center - h_cells)
    r_max = min(grid.shape[0], row_center + h_cells)
    c_min = max(0, col_center - w_cells)
    c_max = min(grid.shape[1], col_center + w_cells)
    
    grid[r_min:r_max, c_min:c_max] = 1

def create_hardcoded_map(cenario):
    grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int) 
    
    draw_wall(grid, 5.0, -1.0, 12.2, 0.2) 
    draw_wall(grid, 5.0, 11.0, 12.2, 0.2) 
    draw_wall(grid, -1.0, 5.0, 0.2, 12.0) 
    draw_wall(grid, 11.0, 5.0, 0.2, 12.0) 
    
    if cenario == 'maze':
        draw_wall(grid, 4.0, 4.0, 2.0, 2.0) 
        draw_wall(grid, 7.0, 6.0, 0.5, 4.0) 
    elif cenario == 'simple':
        draw_wall(grid, 4.5, 4.5, 0.5, 6.0) 

    return grid

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
        self.declare_parameter('scenario', 'simple')
        self.cenario = self.get_parameter('scenario').value

        self.grid = create_hardcoded_map(self.cenario)

        self.origin_x = ORIGIN
        self.origin_y = ORIGIN
        self.resolution = RESOLUTION
        self.grid_height, self.grid_width = self.grid.shape
        self.map_recebido = True
          
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

        # --- INTEGRAÇÃO DO MAPA COMO PARTÍCULAS FIXAS ---
        for r in range(self.grid_height):
            for c in range(self.grid_width):
                if self.grid[r, c] == 1:
                    # Converte os índices da matriz de volta para coordenadas globais
                    obs_x = (c * self.resolution) + self.origin_x
                    obs_y = (r * self.resolution) + self.origin_y
                    
                    p_obs = Particle(id=f"obs_{r}_{c}", x=obs_x, y=obs_y, m=1000.0)
                    p_obs.fixed = True  # Define como obstáculo estático
                    self.particles.append(p_obs)
                    
        self.get_logger().info(f"Mapa carregado: {len(self.particles) - self.num_robots} partículas de parede criadas.")

        # Initialize target potential
        self.pot = Potential(xc=self.goal_x, yc=self.goal_y, R=1)
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

    # --- MAIN CONTROL LOOP --- 
    def control_loop(self):
        
        if len(self.particles) < self.num_robots:
            return
        if not all(self.odom_recebida):
            return

        # 1. ADVANCE SPH (Livre, sem forçar robô) 
        for i in range(self.num_robots): 
            # self.particles[i].x = self.robot_x[i] 
            # self.particles[i].y = self.robot_y[i] 
            self.particles[i].ext_force = self.pot.force(self.particles[i].x, self.particles[i].y) 
            # TESTE PARA COMPORTAMENTO SEM FORÇA
            #self.particles[i].ext_force = [0.0, 0.0] 
            
        # Atualização da física SPH 
        particulas_dinamicas = [p for p in self.particles if not getattr(p, 'fixed', False)] 
        resultados = []
        for p in particulas_dinamicas:
            n_rho = p.next_rho(self.particles)
            n_P = p.next_pressure()
            dvdt = p.next_vel(self.particles)
            e = p.next_energy(self.particles)
            
            # --- TRAVA DE ACELERAÇÃO ---
            # Impede que picos de 600 milhões de pressão gerem acelerações infinitas.
            # Se a aceleração tentar passar de 5.0 m/s², nós seguramos ela.
            dvdt[0] = float(np.clip(dvdt[0], -5.0, 5.0))
            dvdt[1] = float(np.clip(dvdt[1], -5.0, 5.0))
            
            resultados.append((n_rho, n_P, dvdt, e))
        
        for p, (n_rho, n_P, dvdt, e) in zip(particulas_dinamicas, resultados):
            p.update_particle(n_rho, n_P, dvdt, e, self.dt)
                
                # --- TRAVA DE SEGURANÇA ANTISPLOSÃO ---
                # Impede que a partícula seja ejetada para o infinito em cantos fechados
            p.vel[0] = float(np.clip(p.vel[0], -1.5, 1.5))
            p.vel[1] = float(np.clip(p.vel[1], -1.5, 1.5))

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

        # Registro de tempo (Movido para o final para garantir que o plot não quebre se o usuário der Ctrl+C no meio da volta)
        tempo_atual = self.get_clock().now().nanoseconds / 1e9 
        self.hist_tempo.append(tempo_atual)


    def plot_results(self): 
        """Generates, saves and displays historical performance charts for all drones.""" 
        if not self.hist_tempo: 
            self.get_logger().warn("No historical data collected to plot.") 
            return 

        # Normalize time array 
        t = np.array(self.hist_tempo) - self.hist_tempo[0] 

        plt.close('all') 
        plt.figure(figsize=(14, 10)) 

        # --- Graph 1: XY Trajectories over the Map --- 
        plt.subplot(2, 2, 1) 

        # Renderiza a matriz hardcoded como fundo do mapa
        extent_box = [self.origin_x, self.origin_x + (self.grid_width * self.resolution),
                      self.origin_y, self.origin_y + (self.grid_height * self.resolution)]
        
        # O 'cmap=Greys' exibirá o 0 como branco (livre) e 1 como preto (parede)
        plt.imshow(self.grid, origin='lower', extent=extent_box, cmap='Greys', alpha=0.3)
        
        # --- Trajetórias dos robôs ---
        for i in range(self.num_robots):
            plt.plot(self.hist_x_rob[i], self.hist_y_rob[i], label=f'Robô {i}')

        # --- Ponto objetivo ---
        plt.scatter(self.pot.xc, self.pot.yc, color='red', marker='X', s=150, label='Potencial Objetivo')

        # --- Círculo ao redor do ponto ---
        circle = plt.Circle((self.pot.xc, self.pot.yc), self.pot.R,
                            color='red', fill=False, linestyle='--', linewidth=2,
                            label='Raio Potencial', clip_on=False)  # não corta nas bordas
        plt.gca().add_patch(circle)

        # --- Configurações finais ---
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

        plt.subplot(2, 2, 4)
        for i in range(self.num_robots):
            plt.plot(self.hist_x_sph[i], self.hist_y_sph[i], linewidth=1.5, label=f'Partícula {i}')
        plt.title('Trajetória das Partículas SPH')
        plt.xlabel('X (metros)')
        plt.ylabel('Y (metros)')
        plt.grid(True)
        plt.axis('equal')

        plt.tight_layout() 

        # Cria a pasta e salva a imagem substituindo a anterior
        os.makedirs('plots', exist_ok=True)
        plt.savefig('plots/sph.png', dpi=300, bbox_inches='tight')
        self.get_logger().info("Gráfico salvo com sucesso em 'plots/sph.png'")

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