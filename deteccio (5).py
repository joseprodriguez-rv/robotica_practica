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

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_ang = 0.0

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub_laser = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile)

        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.en_maniobra = 0
        self.sub_maniobra = self.create_subscription(
            Int32, '/en_maniobra', self.maniobra_callback, 10)

        self.objectes = 0
        self.sub_comptador = self.create_subscription(
            Int32, '/comptador_objectes', self.comptador_callback, 10)

        self.pub_objecte = self.create_publisher(Odometry, '/objecte_detectat', 10)
        self.pub_tipus   = self.create_publisher(String, '/tipus_obstacle', 10)

        # FIX: debounce per evitar múltiples deteccions del mateix obstacle
        self._cooldown     = 0
        self._COOLDOWN_MAX = 8   # ~0.8 s a 10 Hz

        self.get_logger().info('Node de Detecció actiu')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        self.robot_ang = math.atan2(siny_cosp, cosy_cosp)

    def maniobra_callback(self, msg):
        self.en_maniobra = msg.data

    def comptador_callback(self, msg):
        self.objectes = msg.data

    def classificar_obstacle(self, msg, distancia_min):
        marge = 0.11
        n     = len(msg.ranges)   # FIX: índexs dinàmics adaptats a la resolució real

        # FIX: zona central ±15° (dinàmica) en lloc de [0:30] fix
        idx_c  = int(15 * n / 360)
        idx_c2 = int(15 * n / 360)
        centre = list(msg.ranges[:idx_c]) + list(msg.ranges[n - idx_c2:])
        centre_valids = [d for d in centre if msg.range_min < d < msg.range_max]
        if len(centre_valids) == 0:
            return ''

        propers_centre   = [d for d in centre_valids if d < distancia_min + marge]
        proporcio_centre = len(propers_centre) / len(centre_valids)

        # FIX: laterals a ~75° i ~285° (dinàmics)
        idx_esq_ini = int(60  * n / 360)
        idx_esq_fi  = int(90  * n / 360)
        idx_dre_ini = int(270 * n / 360)
        idx_dre_fi  = int(300 * n / 360)
        lateral_esq = [d for d in msg.ranges[idx_esq_ini:idx_esq_fi]
                       if msg.range_min < d < msg.range_max]
        lateral_dre = [d for d in msg.ranges[idx_dre_ini:idx_dre_fi]
                       if msg.range_min < d < msg.range_max]

        llindar_lateral = 0.5
        # FIX: llindar de punts centrals proporcional a n (era fix >30)
        llindar_propers = int(30 * n / 360)

        esq_bloquejat = (
            len(propers_centre) > llindar_propers and
            len(lateral_esq) > 0 and
            sum(1 for d in lateral_esq if d < llindar_lateral) / len(lateral_esq) > 0.95
        )
        dre_bloquejat = (
            len(propers_centre) > llindar_propers and
            len(lateral_dre) > 0 and
            sum(1 for d in lateral_dre if d < llindar_lateral) / len(lateral_dre) > 0.95
        )

        # FIX: llindar superior també proporcional (era fix <40)
        llindar_objecte_max = int(40 * n / 360)
        es_objecte = 0 < len(propers_centre) < llindar_objecte_max
        es_paret   = (esq_bloquejat or dre_bloquejat) or proporcio_centre > 0.7

        self.get_logger().info(f'És objecte: {es_objecte}. És paret: {es_paret}')

        if es_objecte and not es_paret:
            return 'OBJECTE'
        elif not es_objecte and es_paret:
            return 'PARET'
        else:
            return ''

    def laser_callback(self, msg):
        if self.en_maniobra == 1 or self.objectes >= 5:
            return

        # FIX: debounce actiu
        if self._cooldown > 0:
            self._cooldown -= 1
            return

        n = len(msg.ranges)
        if n == 0:
            return

        part_esquerra = msg.ranges[0:60]
        part_dreta    = msg.ranges[300:360]
        con_frontal   = list(part_esquerra) + list(part_dreta)

        distancies_valides = [d for d in con_frontal if msg.range_min < d < msg.range_max]

        if len(distancies_valides) > 0:
            distancia_min = min(distancies_valides)
            llindar = 0.28 if self.en_maniobra == 2 else 0.33

            if distancia_min < llindar:
                tipus      = String()
                tipus.data = self.classificar_obstacle(msg, distancia_min)

                if tipus.data == 'PARET':
                    self.get_logger().info('PARET detectada')
                    self.pub_tipus.publish(tipus)
                    self._cooldown = self._COOLDOWN_MAX   # FIX: debounce també per PARET

                elif tipus.data == 'OBJECTE':
                    # FIX: argmin manual — evita empat amb .index()
                    num_min = min(
                        range(len(msg.ranges)),
                        key=lambda i: msg.ranges[i]
                        if msg.range_min < msg.ranges[i] < msg.range_max
                        else float('inf')
                    )
                    angle = msg.angle_min + (num_min * msg.angle_increment)
                    self.get_logger().warn(f'Objecte detectat a {distancia_min:.2f}m')
                    self.enviar_posicio_objecte(distancia_min, angle)
                    self.pub_tipus.publish(tipus)
                    self._cooldown = self._COOLDOWN_MAX   # FIX: debounce

                else:
                    pass

    def enviar_posicio_objecte(self, r, a):
        angle_final = self.robot_ang + a
        obj_x = self.robot_x + (r * math.cos(angle_final))
        obj_y = self.robot_y + (r * math.sin(angle_final))

        msg_obj = Odometry()
        msg_obj.header.stamp    = self.get_clock().now().to_msg()
        msg_obj.header.frame_id = 'map'
        msg_obj.pose.pose.position.x = float(obj_x)
        msg_obj.pose.pose.position.y = float(obj_y)

        self.pub_objecte.publish(msg_obj)

def main(args=None):
    rclpy.init(args=args)
    node = DeteccioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Aturant node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
