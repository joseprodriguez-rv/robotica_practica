#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int32
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class DeteccioNode(Node):
    def __init__(self):
        super().__init__('deteccio')

        self.robot_x   = 0.0
        self.robot_y   = 0.0
        self.robot_ang = 0.0

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub_laser     = self.create_subscription(LaserScan, '/scan', self.laser_callback, qos_sensor)
        self.sub_odom      = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_maniobra  = self.create_subscription(Int32, '/en_maniobra', self.maniobra_callback, 10)
        self.sub_comptador = self.create_subscription(Int32, '/comptador_objectes', self.comptador_callback, 10)

        self.pub_objecte = self.create_publisher(Odometry, '/objecte_detectat', 10)
        self.pub_tipus   = self.create_publisher(String, '/tipus_obstacle', 10)

        self.en_maniobra = 0
        self.objectes    = 0

        # Debounce: evitar múltiples deteccions del mateix obstacle en escanades consecutives
        self._cooldown = 0
        self._COOLDOWN_MAX = 8

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
        if self.objectes >= 5:
            return

        # Durant girs (en_maniobra==1) detecció completament desactivada
        # Durant rectes d'esquiva (en_maniobra==2) detecció activa amb llindar reduït
        # Durant exploració (en_maniobra==0) detecció normal
        if self.en_maniobra == 1:
            return

        if self._cooldown > 0:
            self._cooldown -= 1
            return

        n = len(msg.ranges)
        if n == 0:
            return

        llindar = 0.15 if self.en_maniobra == 2 else 0.25

        # CON AMPLIAT: ±90° en lloc de ±60°
        # Permet detectar objectes que estaven "mig amagats" als laterals
        # durant l'exploració i durant les rectes d'esquiva
        graus_con = 90
        idx_con   = int(graus_con * n / 360)
        con = list(msg.ranges[:idx_con]) + list(msg.ranges[n - idx_con:])

        distancies_valides = [d for d in con if msg.range_min < d < msg.range_max]

        if not distancies_valides:
            return

        distancia_min = min(distancies_valides)

        if distancia_min < llindar:
            marge   = 0.10
            propers = [d for d in distancies_valides if d < distancia_min + marge]

            tipus = String()
            # Més de 60 punts propers → paret; menys → objecte petit
            if len(propers) > 60:
                tipus.data = 'PARET'
                self.get_logger().info('PARET detectada')
            else:
                tipus.data = 'OBJECTE'
                num_min = msg.ranges.index(distancia_min)
                angle   = msg.angle_min + num_min * msg.angle_increment
                self.get_logger().warn(
                    f'Objecte detectat a {distancia_min:.2f}m, angle={math.degrees(angle):.1f}°'
                )
                self.enviar_posicio_objecte(distancia_min, angle)
                self._cooldown = self._COOLDOWN_MAX

            self.pub_tipus.publish(tipus)

    def enviar_posicio_objecte(self, r, angle_laser):
        angle_final = self.robot_ang + angle_laser
        obj_x = self.robot_x + r * math.cos(angle_final)
        obj_y = self.robot_y + r * math.sin(angle_final)

        # Descartar posicions (0,0) — odometria no inicialitzada
        if abs(obj_x) < 0.01 and abs(obj_y) < 0.01:
            return

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
