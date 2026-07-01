import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import yaml
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', 10)

        self.declare_parameter('scenario', 'complex')  # valor padrão
        scenario = self.get_parameter('scenario').value
        self.get_logger().info(f"Carregando mapa para o cenário: {scenario}")

        map_files = {
            'maze': 'map_maze',
            'complex': 'map_complex',
            'arena': 'map_arena',
        }
        base_name = map_files.get(scenario, 'map_complex')  # fallback

        pkg_share = get_package_share_directory('SPH')
        yaml_path = os.path.join(pkg_share, 'maps', f'{base_name}.yaml')
        pgm_path = os.path.join(pkg_share, 'maps', f'{base_name}.pgm')

        # # Caminhos dos arquivos (substitua pelos seus caminhos reais)
        # pkg_share = get_package_share_directory('tp2')
        # yaml_path = os.path.join(pkg_share, 'maps', 'map_complex.yaml')
        # pgm_path = os.path.join(pkg_share, 'maps', 'map_complex.pgm')

        self.EXPANSION_CELLS = 1
        
        self.grid_msg = self.load_map(yaml_path, pgm_path)
        
        # Publica o mapa uma vez por segundo
        self.timer = self.create_timer(1.0, self.timer_callback)

    def load_map(self, yaml_path, pgm_path):
        # 1. Ler metadados do YAML
        with open(yaml_path, 'r') as f:
            yaml_data = yaml.safe_load(f)
        
        original_resolution = yaml_data['resolution']
        origin = yaml_data['origin']  # [x, y, yaw]
        negate = yaml_data.get('negate', 0)  # usa o que está no YAML
        occupied_thresh = yaml_data.get('occupied_thresh', 0.65)
        free_thresh = yaml_data.get('free_thresh', 0.19)

        # 2. Ler imagem PGM via OpenCV
        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error(f"Falha ao carregar: {pgm_path}")
            return OccupancyGrid()

        height, width = img.shape

        # 3. Resolução desejada (em metros por célula) – ALTERE AQUI!
        RESOLUTION_DESIRED = 0.5    

        # Calcular fator de redimensionamento
        scale_factor = original_resolution / RESOLUTION_DESIRED
        new_width = int(round(width * scale_factor))
        new_height = int(round(height * scale_factor))

        # Redimensionar a imagem (usando interpolação NEAREST para preservar valores discretos)
        img_resized = cv2.resize(img, (new_width, new_height), interpolation=cv2.INTER_NEAREST)

        # Se negate for True, inverter
        if negate:
            img_resized = 255 - img_resized

        # Normalizar
        data_normalized = img_resized.astype(np.float32) / 255.0

        # Criar grid (agora com as novas dimensões)
        grid_data = np.zeros((new_height, new_width), dtype=np.int8)

        # Thresholds (usando os mesmos valores do YAML)
        grid_data[data_normalized > (1.0 - free_thresh)] = 0          # Livre
        grid_data[data_normalized < (1.0 - occupied_thresh)] = 100    # Ocupado
        #grid_data[(data_normalized >= (1.0 - occupied_thresh)) & (data_normalized <= (1.0 - free_thresh))] = -1  # Desconhecido
        grid_data[(data_normalized >= (1.0 - occupied_thresh)) & (data_normalized <= (1.0 - free_thresh))] = 100  # Desconhecido

        
        # Máscara das células ocupadas (valor 100)
        obstacle_mask = (grid_data == 100)
        mask_uint8 = obstacle_mask.astype(np.uint8) * 255

        # Kernel quadrado de lado (2*d + 1)
        kernel_size = 2 * self.EXPANSION_CELLS + 1
        kernel = np.ones((kernel_size, kernel_size), np.uint8)

        # Dilatação
        dilated_mask = cv2.dilate(mask_uint8, kernel, iterations=1)

        # Atualiza a grade: células dilatadas tornam-se ocupadas (100)
        grid_data[dilated_mask > 0] = 100

        # ROS espera origem no canto inferior esquerdo
        grid_data = np.flipud(grid_data)

        # Construir mensagem
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.info.resolution = RESOLUTION_DESIRED
        msg.info.width = new_width
        msg.info.height = new_height

        # A origem (posição do canto inferior esquerdo) deve ser ajustada para manter o mesmo frame do mundo.
        # Como redimensionamos, a origem em coordenadas do mundo continua a mesma.
        pose = Pose()
        pose.position.x = float(origin[0])
        pose.position.y = float(origin[1])
        pose.position.z = 0.0
        from geometry_msgs.msg import Quaternion
        import math
        half_yaw = float(origin[2]) * 0.5
        pose.orientation = Quaternion(x=0.0, y=0.0, z=math.sin(half_yaw), w=math.cos(half_yaw))
        msg.info.origin = pose

        msg.data = grid_data.flatten().tolist()
        return msg

    def timer_callback(self):
        self.grid_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.grid_msg)
        # self.get_logger().info('OccupancyGrid publicado com sucesso.')

def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
