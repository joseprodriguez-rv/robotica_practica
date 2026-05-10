#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32, String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math

# ─── Paràmetres ajustables ──────────────────────────────────────────────────
VEL_LINEAL   = 0.15    # m/s — velocitat d'avanç
VEL_GIR_MAX  = 0.40    # rad/s — velocitat màxima de gir
VEL_GIR_MIN  = 0.07    # rad/s — velocitat mínima de gir (per no aturar-se)
DIST_LATERAL = 0.28    # m — distància lateral per esquivar
DIST_SUPERAR = 0.45    # m — distància frontal per superar l'objecte
TOL_GIR      = 3.0     # graus — tolerància per considerar un gir acabat
KP_DERIVA    = 1.5     # proporcional corrector de deriva en recta
# ────────────────────────────────────────────────────────────────────────────

def angle_diff(a, b):
    """Diferència normalitzada a [-pi, pi], gestiona wrap-around."""
    d = a - b
    while d >  math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


class MovimentNode(Node):
    def __init__(self):
        super().__init__('controlador_moviment')

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_moviment = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub_laser     = self.create_subscription(LaserScan, '/scan',                self.laser_callback,     qos_sensor)
        self.sub_odom      = self.create_subscription(Odometry,  '/odom',               self.odom_callback,      10)
        self.sub_comptador = self.create_subscription(Int32,     '/comptador_objectes', self.comptador_callback,  10)
        self.sub_tipus     = self.create_subscription(String,    '/tipus_obstacle',     self.tipus_callback,      10)
        self.timer         = self.create_timer(0.1, self.control_callback)

        self.pub          = self.create_publisher(TwistStamped, '/cmd_vel',     qos_moviment)
        # Int32: 0=explorant, 1=girant (detecció OFF), 2=avançant en esquiva (llindar reduït)
        self.pub_maniobra = self.create_publisher(Int32,         '/en_maniobra', 10)

        # ── Màquina d'estats ─────────────────────────────────────────────
        self.estat          = 0
        self.estat_loguejat = None
        self.objectes       = 0
        self.tipus_obstacle = None

        # Costat decidit pel laser (no canvia durant tota la maniobra)
        self.direccio_esquivar = 1   # +1=esquerra, -1=dreta
        self.direccio_paret    = 1

        # ── LiDAR ────────────────────────────────────────────────────────
        self.laser_ranges    = []
        self.laser_range_min = 0.1
        self.laser_range_max = 10.0

        # ── Odometria — girs amb yaw absolut (evita wrap-around) ─────────
        self.yaw_actual   = 0.0
        self.yaw_objectiu = None   # None = no hi ha gir actiu

        # ── Odometria — rectes amb posició ───────────────────────────────
        self.robot_x       = 0.0
        self.robot_y       = 0.0
        self.x_inici_recta = 0.0
        self.y_inici_recta = 0.0
        self.yaw_inici_recta = None   # per corregir deriva en recta

        self.odom_rebuda = False   # True quan arriba el primer missatge real

        self.noms_estat = {
            None: 'FINAL',
            0:  'EXPLORAR',
            1:  'PARET — Gir 135°',
            10: 'ESQUIVA — Gir 90° costat lliure',
            11: 'ESQUIVA — Avançar lateral',
            12: 'ESQUIVA — Gir 90° contrari',
            13: 'ESQUIVA — Avançar superar objecte',
            14: 'ESQUIVA — Gir 90° contrari (2)',
            15: 'ESQUIVA — Avançar tornar ruta',
            16: 'ESQUIVA — Gir 90° redreçar',
        }
        self.get_logger().info('Node de moviment actiu...')

    # ── Logging ───────────────────────────────────────────────────────────

    def log_estat(self):
        if self.estat != self.estat_loguejat:
            nom = self.noms_estat.get(self.estat, f'DESCONEGUT ({self.estat})')
            self.get_logger().info(f'[ESTAT {self.estat}] {nom}')
            self.estat_loguejat = self.estat

    # ── Callbacks ─────────────────────────────────────────────────────────

    def laser_callback(self, msg):
        self.laser_ranges    = list(msg.ranges)
        self.laser_range_min = msg.range_min
        self.laser_range_max = msg.range_max

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q    = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw_actual = math.atan2(siny, cosy)
        if not self.odom_rebuda:
            self.odom_rebuda     = True
            self.yaw_inici_recta = self.yaw_actual
            self.get_logger().info(f'Odometria inicial: yaw={math.degrees(self.yaw_actual):.1f}°')

    def comptador_callback(self, msg):
        self.objectes = msg.data
        if self.objectes >= 5:
            self.estat = None
            self.get_logger().info('Objectiu complert: 5 objectes trobats!')
            self._aturar()

    def tipus_callback(self, msg):
        # Durant girs, el laser apunta en direccions canviants → ignorar
        if self.estat in (1, 10, 12, 14, 16):
            return
        if self.estat is None:
            return
        # Durant avanços (11, 13, 15) i recta (0): acceptar
        self.tipus_obstacle = msg.data

    # ── Helpers de gir ────────────────────────────────────────────────────

    def inici_gir(self, graus):
        """Guarda el yaw OBJECTIU absolut. Gestiona wrap-around."""
        self.yaw_objectiu = self.yaw_actual + math.radians(graus)
        while self.yaw_objectiu >  math.pi: self.yaw_objectiu -= 2 * math.pi
        while self.yaw_objectiu < -math.pi: self.yaw_objectiu += 2 * math.pi

    def gir_acabat(self):
        if self.yaw_objectiu is None:
            return True
        return abs(angle_diff(self.yaw_actual, self.yaw_objectiu)) < math.radians(TOL_GIR)

    def vel_gir(self, signe):
        """Velocitat proporcional a l'error restant (frenada suau al final)."""
        if self.yaw_objectiu is None:
            return 0.0
        error = abs(angle_diff(self.yaw_actual, self.yaw_objectiu))
        vel = min(VEL_GIR_MAX, max(VEL_GIR_MIN, error * (VEL_GIR_MAX / math.radians(25))))
        return signe * vel

    # ── Helpers de recta ──────────────────────────────────────────────────

    def inici_recta(self):
        self.x_inici_recta   = self.robot_x
        self.y_inici_recta   = self.robot_y
        self.yaw_inici_recta = self.yaw_actual

    def distancia_recorreguda(self):
        dx = self.robot_x - self.x_inici_recta
        dy = self.robot_y - self.y_inici_recta
        return math.sqrt(dx * dx + dy * dy)

    def correccio_deriva(self):
        """Petita correcció angular per mantenir la direcció de la recta."""
        if self.yaw_inici_recta is None:
            return 0.0
        return -KP_DERIVA * angle_diff(self.yaw_actual, self.yaw_inici_recta)

    # ── Helpers de LiDAR ──────────────────────────────────────────────────

    def calcular_costat_lliure(self):
        """
        Compara els punts vàlids als sectors laterals.
        Índexs per a Lidar 360°, antihorari:
          [60:120]  → esquerra real (~90°)
          [240:300] → dreta real (~270°)
        Retorna +1 (esquerra) o -1 (dreta).
        angular.z > 0 = gir esquerra en ROS2.
        """
        if len(self.laser_ranges) < 360:
            return 1

        esquerra = self.laser_ranges[60:120]
        dreta    = self.laser_ranges[240:300]

        num_esq = len([d for d in esquerra if 0.1 < d < 6.0])
        num_dre = len([d for d in dreta    if 0.1 < d < 6.0])

        self.get_logger().info(f'Costat lliure: ESQ={num_esq} punts | DRE={num_dre} punts')

        if num_esq >= num_dre:
            self.get_logger().info('→ ESQUERRA (+1)')
            return 1
        else:
            self.get_logger().info('→ DRETA (-1)')
            return -1

    def _aturar(self):
        cmd = TwistStamped()
        cmd.header.stamp    = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        self.pub.publish(cmd)

    # ── Gestió d'obstacles ────────────────────────────────────────────────

    def comprovar_obstacle(self, estat_seguent):
        """Decideix l'estat següent en acabar un segment."""
        self.get_logger().info(
            f'[COMPROVAR] tipus={self.tipus_obstacle!r} estat={self.estat} → {estat_seguent}')

        if self.tipus_obstacle == 'PARET':
            self.get_logger().warn('PARET → Girant 135°')
            self.direccio_paret = self.calcular_costat_lliure()
            self.tipus_obstacle = None
            self.estat          = 1
            self.inici_gir(135 * self.direccio_paret)

        elif self.tipus_obstacle == 'OBJECTE':
            if self.estat >= 10:
                # Ja estem esquivant: no reiniciem, continuem al segment següent
                self.get_logger().info('Objecte durant esquiva → continuant maniobra')
                self.tipus_obstacle = None
                self.estat          = estat_seguent
                if estat_seguent in (10, 12, 14, 16):
                    graus = {
                        10:  90 * self.direccio_esquivar,
                        12: -90 * self.direccio_esquivar,
                        14: -90 * self.direccio_esquivar,
                        16:  90 * self.direccio_esquivar,
                    }[estat_seguent]
                    self.inici_gir(graus)
                else:
                    self.inici_recta()
            else:
                self.get_logger().warn('OBJECTE → Esquivant')
                self.direccio_esquivar = self.calcular_costat_lliure()
                self.tipus_obstacle    = None
                self.estat             = 10
                self.inici_gir(90 * self.direccio_esquivar)

        else:
            self.tipus_obstacle = None
            self.estat          = estat_seguent
            if estat_seguent in (1, 10, 12, 14, 16):
                graus = {
                    1:   135 * self.direccio_paret,
                    10:   90 * self.direccio_esquivar,
                    12:  -90 * self.direccio_esquivar,
                    14:  -90 * self.direccio_esquivar,
                    16:   90 * self.direccio_esquivar,
                }.get(estat_seguent, 0)
                self.inici_gir(graus)
            else:
                self.inici_recta()

    # ── Bucle de control (10 Hz) ──────────────────────────────────────────

    def control_callback(self):
        cmd = TwistStamped()
        cmd.header.stamp    = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        self.log_estat()

        # Flag maniobra per a deteccio.py
        flag = Int32()
        if self.estat in (1, 10, 12, 14, 16):
            flag.data = 1   # girant → detecció OFF
        elif self.estat in (11, 13, 15):
            flag.data = 2   # avançant en esquiva → llindar reduït
        else:
            flag.data = 0   # explorant → detecció normal
        self.pub_maniobra.publish(flag)

        if self.estat is None:
            self._aturar()
            return

        # ── ESTAT 0: EXPLORAR ─────────────────────────────────────────────
        if self.estat == 0:
            if not self.odom_rebuda:
                self.get_logger().info('Esperant odometria...', throttle_duration_sec=1.0)
                self.pub.publish(cmd)
                return
            if self.yaw_inici_recta is None:
                self.inici_recta()
            if self.tipus_obstacle is None:
                cmd.twist.linear.x  = VEL_LINEAL
                cmd.twist.angular.z = self.correccio_deriva()
            else:
                self.comprovar_obstacle(0)

        # ── ESTAT 1: Gir 135° per PARET ──────────────────────────────────
        elif self.estat == 1:
            if self.gir_acabat():
                self.get_logger().info('Gir paret acabat')
                self.comprovar_obstacle(0)
            else:
                cmd.twist.angular.z = self.vel_gir(self.direccio_paret)

        # ── ESTATS 10-16: Esquiva en C ────────────────────────────────────

        elif self.estat == 10:   # Gir 90° cap al costat lliure
            if self.gir_acabat():
                self.comprovar_obstacle(11)
            else:
                cmd.twist.angular.z = self.vel_gir(self.direccio_esquivar)

        elif self.estat == 11:   # Avançar lateral
            if self.tipus_obstacle is not None:
                cmd.twist.linear.x = 0.0
                self.get_logger().warn('Obstacle durant avanç lateral → gestionant')
                self.comprovar_obstacle(11)
            elif self.distancia_recorreguda() < DIST_LATERAL:
                cmd.twist.linear.x  = VEL_LINEAL
                cmd.twist.angular.z = self.correccio_deriva()
            else:
                self.comprovar_obstacle(12)

        elif self.estat == 12:   # Gir 90° cap al costat contrari
            if self.gir_acabat():
                self.comprovar_obstacle(13)
            else:
                cmd.twist.angular.z = self.vel_gir(-self.direccio_esquivar)

        elif self.estat == 13:   # Avançar per superar l'objecte
            if self.tipus_obstacle is not None:
                cmd.twist.linear.x = 0.0
                self.get_logger().warn('Obstacle durant avanç superar → gestionant')
                self.comprovar_obstacle(13)
            elif self.distancia_recorreguda() < DIST_SUPERAR:
                cmd.twist.linear.x  = VEL_LINEAL
                cmd.twist.angular.z = self.correccio_deriva()
            else:
                self.comprovar_obstacle(14)

        elif self.estat == 14:   # Gir 90° cap al costat contrari (2)
            if self.gir_acabat():
                self.comprovar_obstacle(15)
            else:
                cmd.twist.angular.z = self.vel_gir(-self.direccio_esquivar)

        elif self.estat == 15:   # Avançar per tornar a la ruta
            if self.tipus_obstacle is not None:
                cmd.twist.linear.x = 0.0
                self.get_logger().warn('Obstacle durant avanç tornar → gestionant')
                self.comprovar_obstacle(15)
            elif self.distancia_recorreguda() < DIST_LATERAL:
                cmd.twist.linear.x  = VEL_LINEAL
                cmd.twist.angular.z = self.correccio_deriva()
            else:
                self.comprovar_obstacle(16)

        elif self.estat == 16:   # Gir 90° per redreçar
            if self.gir_acabat():
                self.get_logger().info('Esquiva acabada → tornant a explorar')
                self.tipus_obstacle = None
                self.comprovar_obstacle(0)
            else:
                cmd.twist.angular.z = self.vel_gir(self.direccio_esquivar)

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MovimentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Aturant...')
        node._aturar()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
