#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int32
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ─── Paràmetres ajustables ──────────────────────────────────────────────────
LLINDAR_NORMAL  = 0.30   # m — detecció en exploració
LLINDAR_ESQUIVA = 0.25   # m — detecció durant avanços laterals (més estricte)
PUNTS_PARET     = 75     # # punts mínims per classificar com PARET
SEMIANG_CON     = 60     # índexs a cada costat del frontal (120° total)
# ────────────────────────────────────────────────────────────────────────────

class DeteccioNode(Node):
    def __init__(self):
        super().__init__('deteccio')

        self.robot_x   = 0.0
        self.robot_y   = 0.0
        self.robot_ang = 0.0

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub_laser     = self.create_subscription(LaserScan, '/scan',                self.laser_callback,     qos_profile)
        self.sub_odom      = self.create_subscription(Odometry,  '/odom',               self.odom_callback,      10)
        self.sub_maniobra  = self.create_subscription(Int32,     '/en_maniobra',        self.maniobra_callback,  10)
        self.sub_comptador = self.create_subscription(Int32,     '/comptador_objectes', self.comptador_callback, 10)

        self.pub_objecte = self.create_publisher(Odometry, '/objecte_detectat', 10)
        self.pub_tipus   = self.create_publisher(String,   '/tipus_obstacle',   10)

        # 0=explorant, 1=girant (OFF), 2=avançant esquiva (llindar reduït)
        self.en_maniobra = 0
        self.objectes    = 0

        self.get_logger().info('Node de Detecció actiu')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.robot_ang = math.atan2(
            2 * (qw * qz + qx * qy),
            1 - 2 * (qy * qy + qz * qz)
        )

    def maniobra_callback(self, msg):
        self.en_maniobra = msg.data

    def comptador_callback(self, msg):
        self.objectes = msg.data

    def laser_callback(self, msg):
        # Durant girs: detecció completament desactivada
        if self.en_maniobra == 1 or self.objectes >= 5:
            return

        # Con frontal: SEMIANG_CON índexs a cada costat del 0 (davant)
        con_frontal = list(msg.ranges[:SEMIANG_CON]) + list(msg.ranges[-SEMIANG_CON:])

        valids = [d for d in con_frontal
                  if msg.range_min < d < msg.range_max
                  and not math.isnan(d) and not math.isinf(d)]

        if not valids:
            return

        dist_min = min(valids)
        llindar  = LLINDAR_ESQUIVA if self.en_maniobra == 2 else LLINDAR_NORMAL

        if dist_min >= llindar:
            return

        # ── Classificació simple i robusta ───────────────────────────────
        # Paret: molts punts al con frontal propers al mínim
        # Objecte: pocs punts (ampolla, estoig, cilindre)
        marge   = 0.10
        propers = [d for d in valids if d < dist_min + marge]

        tipus = String()
        if len(propers) >= PUNTS_PARET:
            tipus.data = 'PARET'
            self.get_logger().info(f'PARET detectada ({len(propers)} punts, dist={dist_min:.2f}m)')
        else:
            tipus.data = 'OBJECTE'
            # Buscar l'índex del mínim en el rang complet per calcular l'angle
            idx_min = None
            min_val = float('inf')
            for i in list(range(SEMIANG_CON)) + list(range(len(msg.ranges) - SEMIANG_CON, len(msg.ranges))):
                d = msg.ranges[i]
                if msg.range_min < d < msg.range_max and not math.isnan(d) and d < min_val:
                    min_val = d
                    idx_min = i
            if idx_min is not None:
                angle = msg.angle_min + idx_min * msg.angle_increment
                self.get_logger().warn(
                    f'OBJECTE detectat ({len(propers)} punts, dist={dist_min:.2f}m, angle={math.degrees(angle):.1f}°)')
                self.enviar_posicio_objecte(dist_min, angle)

        self.pub_tipus.publish(tipus)

    def enviar_posicio_objecte(self, r, a):
        angle_final = self.robot_ang + a
        obj_x = self.robot_x + r * math.cos(angle_final)
        obj_y = self.robot_y + r * math.sin(angle_final)

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
