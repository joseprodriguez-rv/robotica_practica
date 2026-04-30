#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool, Int32
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class DeteccioNode(Node):
    def __init__(self):
        super().__init__('deteccio')

        # Posició i orientació del robot
        self.robot_x   = 0.0
        self.robot_y   = 0.0
        self.robot_ang = 0.0

        # QoS per al sensor LiDAR (BEST_EFFORT per compatibilitat amb drivers reals)
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub_laser    = self.create_subscription(LaserScan, '/scan', self.laser_callback, qos_sensor)
        self.sub_odom     = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_maniobra = self.create_subscription(Bool, '/en_maniobra', self.maniobra_callback, 10)
        self.sub_comptador = self.create_subscription(Int32, '/comptador_objectes', self.comptador_callback, 10)

        self.pub_objecte = self.create_publisher(Odometry, '/objecte_detectat', 10)
        self.pub_tipus   = self.create_publisher(String, '/tipus_obstacle', 10)

        self.en_maniobra = False
        self.objectes    = 0

        # Debounce: evitar publicar múltiples deteccions de la mateixa escombrada
        self._cooldown_cicles    = 0
        self._cooldown_max       = 8   # ~0.8 s a 10 Hz de laser

        self.get_logger().info('Node de Detecció actiu')

    # ── Callbacks ──────────────────────────────────────────────────────────

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.robot_ang = math.atan2(siny_cosp, cosy_cosp)

    def maniobra_callback(self, msg):
        self.en_maniobra = msg.data

    def comptador_callback(self, msg):
        self.objectes = msg.data

    def laser_callback(self, msg):
        # Parar detecció durant maniobres o quan ja tenim els 5 objectes
        if self.en_maniobra or self.objectes >= 5:
            return

        # Debounce: esperar uns cicles entre deteccions consecutives
        if self._cooldown_cicles > 0:
            self._cooldown_cicles -= 1
            return

        # Con frontal ±60° (índexs dinàmics per a qualsevol resolució de LiDAR)
        n = len(msg.ranges)
        if n == 0:
            return

        graus_con = 60
        idx_con   = int(graus_con * n / 360)
        con_frontal = list(msg.ranges[:idx_con]) + list(msg.ranges[n - idx_con:])

        distancies_valides = [
            d for d in con_frontal
            if msg.range_min < d < msg.range_max
        ]

        if not distancies_valides:
            return

        distancia_min = min(distancies_valides)

        if distancia_min < 0.4:
            marge   = 0.10
            propers = [d for d in distancies_valides if d < distancia_min + marge]

            tipus = String()
            # Llindar proporcional: ~10° de l'arc frontal indica una paret plana
            llindar_paret = int(10 * n / 360)
            if len(propers) > llindar_paret:
                tipus.data = 'PARET'
                self.get_logger().info('PARET detectada')
                self._cooldown_cicles = self._cooldown_max   # activar debounce també per PARET
            else:
                tipus.data = 'OBJECTE'
                # Índex real del punt més proper (argmin manual, evita empats amb .index())
                num_min = min(range(len(msg.ranges)), key=lambda i: msg.ranges[i]
                              if msg.range_min < msg.ranges[i] < msg.range_max else float('inf'))
                angle   = msg.angle_min + num_min * msg.angle_increment
                self.get_logger().warn(f'Objecte detectat a {distancia_min:.2f} m, angle={math.degrees(angle):.1f}°')
                self.enviar_posicio_objecte(distancia_min, angle)
                self._cooldown_cicles = self._cooldown_max   # activar debounce

            self.pub_tipus.publish(tipus)

    # ── Càlcul de posició mundial de l'objecte ─────────────────────────────

    def enviar_posicio_objecte(self, r, angle_laser):
        """Projecta la distància LiDAR a coordenades del mapa."""
        angle_final = self.robot_ang + angle_laser

        obj_x = self.robot_x + r * math.cos(angle_final)
        obj_y = self.robot_y + r * math.sin(angle_final)

        msg_obj = Odometry()
        msg_obj.header.stamp    = self.get_clock().now().to_msg()
        msg_obj.header.frame_id = 'map'
        msg_obj.pose.pose.position.x = float(obj_x)
        msg_obj.pose.pose.position.y = float(obj_y)

        self.pub_objecte.publish(msg_obj)
        self.get_logger().info(f'Posició objecte → X={obj_x:.2f}, Y={obj_y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = DeteccioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Aturant node de detecció...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
