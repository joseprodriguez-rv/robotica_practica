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

def angle_diff(a, b):
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

        self.sub_laser     = self.create_subscription(LaserScan, '/scan', self.laser_callback, qos_sensor)
        self.sub_odom      = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_comptador = self.create_subscription(Int32, '/comptador_objectes', self.comptador_callback, 10)
        self.sub_tipus     = self.create_subscription(String, '/tipus_obstacle', self.tipus_callback, 10)
        self.timer         = self.create_timer(0.1, self.control_callback)

        self.pub          = self.create_publisher(TwistStamped, '/cmd_vel', qos_moviment)
        self.pub_maniobra = self.create_publisher(Int32, '/en_maniobra', 10)

        # Estat
        self.estat             = 0
        self.cicles            = 0
        self.objectes          = 0
        self.tipus_obstacle    = None
        self.direccio_s        = 1
        self.direccio_esquivar = 1
        self._dir_esq_original = 1

        # LiDAR
        self.laser_ranges    = []
        self.laser_range_min = 0.1
        self.laser_range_max = 10.0

        # Odometria
        self.yaw_actual    = 0.0
        self.yaw_objectiu  = None
        self.robot_x       = 0.0
        self.robot_y       = 0.0
        self.x_inici_recta = 0.0
        self.y_inici_recta = 0.0

        self._cicles_desde_maniobra = 0

        # FIX #3: comptador de lectures consecutives d'obstacle frontal
        # per evitar salts per soroll LiDAR
        self._cicles_obstacle_frontal = 0
        # Llindar: 3 lectures seguides (~0.3 s) per confirmar obstacle real
        self._LLINDAR_OBSTACLE_FRONTAL = 3

        self.get_logger().info('Node de moviment actiu...')

    # ── Callbacks ────────────────────────────────────────────────────────

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.yaw_actual = math.atan2(siny_cosp, cosy_cosp)

    def laser_callback(self, msg):
        self.laser_ranges    = msg.ranges
        self.laser_range_min = msg.range_min
        self.laser_range_max = msg.range_max

    def comptador_callback(self, msg):
        self.objectes = msg.data
        if self.objectes >= 5:
            self.estat = None
            self.get_logger().info('Objectiu complert: 5 objectes trobats!')
            # FIX #10: parar net immediatament sense esperar el proper cicle
            self._publicar_aturada()

    def tipus_callback(self, msg):
        if self.estat not in (0, None):
            return
        if self._cicles_desde_maniobra < 5:
            return
        self.tipus_obstacle = msg.data

    # ── Helpers ──────────────────────────────────────────────────────────

    def inici_recta(self):
        self.x_inici_recta = self.robot_x
        self.y_inici_recta = self.robot_y

    def distancia_recorreguda(self):
        return math.sqrt(
            (self.robot_x - self.x_inici_recta) ** 2 +
            (self.robot_y - self.y_inici_recta) ** 2
        )

    def inici_gir(self, graus):
        self.yaw_objectiu = self.yaw_actual + math.radians(graus)
        while self.yaw_objectiu >  math.pi: self.yaw_objectiu -= 2 * math.pi
        while self.yaw_objectiu < -math.pi: self.yaw_objectiu += 2 * math.pi

    def gir_acabat(self, tolerancia_deg=2.0):
        if self.yaw_objectiu is None:
            return False
        return abs(angle_diff(self.yaw_actual, self.yaw_objectiu)) < math.radians(tolerancia_deg)

    def distancia_con(self, graus_centre, finestra=15):
        """Distància mínima en un con de ±finestra° al voltant de graus_centre."""
        if not self.laser_ranges:
            return 999.0
        n     = len(self.laser_ranges)
        idx_c = int(graus_centre * n / 360) % n
        idx_f = max(1, int(finestra * n / 360))
        idxs  = [(idx_c + i) % n for i in range(-idx_f, idx_f + 1)]
        vals  = [self.laser_ranges[i] for i in idxs
                 if self.laser_range_min < self.laser_ranges[i] < self.laser_range_max]
        return min(vals) if vals else 999.0

    def obstacle_frontal_confirmat(self, llindar=0.20):
        """FIX #3: retorna True només si hi ha obstacle frontal durant
        _LLINDAR_OBSTACLE_FRONTAL cicles consecutius (evita falsos positius per soroll)."""
        if self.distancia_con(0, finestra=20) < llindar:
            self._cicles_obstacle_frontal += 1
        else:
            self._cicles_obstacle_frontal = 0
        return self._cicles_obstacle_frontal >= self._LLINDAR_OBSTACLE_FRONTAL

    def _publicar_aturada(self):
        cmd = TwistStamped()
        cmd.header.stamp    = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x  = 0.0
        cmd.twist.angular.z = 0.0
        self.pub.publish(cmd)

    # ── Control principal (10 Hz) ─────────────────────────────────────────

    def control_callback(self):
        cmd = TwistStamped()
        cmd.header.stamp    = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        # Flag maniobra
        if self.estat in (0, None):
            estat_maniobra = 0
        elif self.estat in (11, 13, 15):
            estat_maniobra = 2
        else:
            estat_maniobra = 1
        flag      = Int32()
        flag.data = estat_maniobra
        self.pub_maniobra.publish(flag)

        if self.estat == 0:
            self._cicles_desde_maniobra += 1
        else:
            self._cicles_desde_maniobra = 0

        # ── ESTAT FINAL ──
        if self.estat is None:
            self._publicar_aturada()
            return

        # ── ESTAT 0: EXPLORAR ────────────────────────────────────────────
        if self.estat == 0:
            if self.tipus_obstacle is None:
                cmd.twist.linear.x  = 0.2
                cmd.twist.angular.z = 0.0
            else:
                cmd.twist.linear.x  = 0.0
                cmd.twist.angular.z = 0.0

                if self.tipus_obstacle == 'PARET':
                    self.get_logger().warn('PARET → Alineant i fent maniobra S')
                    self.estat  = 4
                    self.cicles = 0
                else:
                    self.get_logger().warn('OBJECTE → Esquivant')
                    if len(self.laser_ranges) >= 360:
                        n       = len(self.laser_ranges)
                        idx_60  = int(60  * n / 360)
                        idx_300 = int(300 * n / 360)
                        esq = [d for d in self.laser_ranges[:idx_60]  if 0.1 < d < 6.0]
                        dre = [d for d in self.laser_ranges[idx_300:] if 0.1 < d < 6.0]
                        self.direccio_esquivar = -1 if len(esq) < len(dre) else 1
                        self.get_logger().info(
                            f'ESQ={len(esq)} DRE={len(dre)} → '
                            f'{"ESQUERRA" if self.direccio_esquivar==-1 else "DRETA"}'
                        )
                    else:
                        self.direccio_esquivar = 1

                    self._dir_esq_original = self.direccio_esquivar
                    self._cicles_obstacle_frontal = 0
                    self.inici_gir(90 * self.direccio_esquivar)
                    self.estat  = 10
                    self.cicles = 0

                self.tipus_obstacle = None

        # ── ESTAT 4: ALINEACIÓ PARAL·LELA A LA PARET ─────────────────────
        elif self.estat == 4:
            if len(self.laser_ranges) >= 360:
                n       = len(self.laser_ranges)
                idx_esq = int(45  * n / 360)
                idx_dre = int(315 * n / 360)
                d_esq   = self.laser_ranges[idx_esq]
                d_dre   = self.laser_ranges[idx_dre]
                valids  = (
                    self.laser_range_min < d_esq < self.laser_range_max and
                    self.laser_range_min < d_dre < self.laser_range_max
                )
                if not valids:
                    self.estat  = 1
                    self.cicles = 0
                    self.inici_gir(90 * self.direccio_s)
                else:
                    diff = d_esq - d_dre
                    if abs(diff) < 0.05:
                        self.get_logger().info('Alineat → Iniciant maniobra S')
                        self.estat  = 1
                        self.cicles = 0
                        self.inici_gir(90 * self.direccio_s)
                    elif diff > 0:
                        cmd.twist.angular.z =  0.2
                    else:
                        cmd.twist.angular.z = -0.2
            else:
                self.estat  = 1
                self.cicles = 0
                self.inici_gir(90 * self.direccio_s)

        # ── MANIOBRA EN "S" ───────────────────────────────────────────────

        elif self.estat == 1:
            self.cicles  += 1
            gir_prou      = self.gir_acabat(tolerancia_deg=5.0)
            graus_lat     = 90 if self.direccio_s > 0 else 270
            d_lat         = self.distancia_con(graus_lat, finestra=15)
            paret_visible = d_lat < 0.8
            timeout       = self.cicles > 60

            if gir_prou and (paret_visible or timeout):
                if timeout and not paret_visible:
                    self.get_logger().warn('Estat 1: timeout — continuem sense veure paret')
                else:
                    self.get_logger().info(f'Paret al lateral ({d_lat:.2f} m) → avançant fila')
                self.estat  = 2
                self.cicles = 0
                self.inici_recta()
            else:
                cmd.twist.angular.z = 0.5 * self.direccio_s

        elif self.estat == 2:
            self.cicles += 1
            d_post = self.distancia_con(180, finestra=20)
            dist   = self.distancia_recorreguda()
            # FIX #1: fallback de distància màxima per si el LiDAR posterior queda tapat
            if (dist >= 0.30 and d_post > 0.40) or dist >= 0.60:
                if dist >= 0.60:
                    self.get_logger().warn('Estat 2: fallback distància màxima assolit')
                else:
                    self.get_logger().info(f'Fila canviada (dist={dist:.2f} m)')
                self.estat  = 3
                self.cicles = 0
                self.inici_gir(90 * self.direccio_s)
            else:
                cmd.twist.linear.x = 0.15

        elif self.estat == 3:
            self.cicles  += 1
            gir_prou      = self.gir_acabat(tolerancia_deg=5.0)
            d_front       = self.distancia_con(0, finestra=30)
            paret_visible = d_front < 0.8
            timeout       = self.cicles > 60

            if gir_prou and (paret_visible or timeout):
                if timeout and not paret_visible:
                    self.get_logger().warn('Estat 3: timeout — continuem sense veure paret')
                else:
                    self.get_logger().info(f'Paret al davant ({d_front:.2f} m) → nova passada')
                self.direccio_s    *= -1
                self.estat          = 0
                self.cicles         = 0
                self.tipus_obstacle = None
                self._cicles_desde_maniobra = 0
            else:
                cmd.twist.angular.z = 0.5 * self.direccio_s

        # ── ESQUIVAR OBJECTE ──────────────────────────────────────────────

        elif self.estat == 10:
            if self.gir_acabat():
                self.estat  = 11
                self.cicles = 0
                self._cicles_obstacle_frontal = 0
                self.inici_recta()
            else:
                cmd.twist.angular.z = 0.5 * self._dir_esq_original

        elif self.estat == 11:
            graus_obj = 270 if self._dir_esq_original == 1 else 90
            d_obj     = self.distancia_con(graus_obj, finestra=20)
            dist      = self.distancia_recorreguda()
            # FIX #3: obstacle frontal confirmat per evitar falsos positius
            # FIX #2: fallback de distància màxima per objectes grans
            if self.obstacle_frontal_confirmat(0.20) and dist >= 0.15:
                self.get_logger().warn('Obstacle frontal confirmat durant lateral → forçant gir')
                self._cicles_obstacle_frontal = 0
                self.estat  = 12
                self.cicles = 0
                self.inici_gir(-90 * self._dir_esq_original)
            elif (dist >= 0.25 and d_obj > 0.40) or dist >= 0.60:
                if dist >= 0.60:
                    self.get_logger().warn('Estat 11: fallback distància màxima assolit')
                else:
                    self.get_logger().info(f'Objecte superat lateralment ({d_obj:.2f} m)')
                self._cicles_obstacle_frontal = 0
                self.estat  = 12
                self.cicles = 0
                self.inici_gir(-90 * self._dir_esq_original)
            else:
                cmd.twist.linear.x = 0.2

        elif self.estat == 12:
            if self.gir_acabat():
                self.estat  = 13
                self.cicles = 0
                self._cicles_obstacle_frontal = 0
                self.inici_recta()
            else:
                cmd.twist.angular.z = -0.5 * self._dir_esq_original

        elif self.estat == 13:
            d_front = self.distancia_con(0, finestra=25)
            dist    = self.distancia_recorreguda()
            # FIX #3: obstacle frontal confirmat (no soroll puntual)
            if self.obstacle_frontal_confirmat(0.20) and dist >= 0.10:
                self.get_logger().warn('Obstacle frontal confirmat a est.13 → forçant pas a 14')
                self._cicles_obstacle_frontal = 0
                self.estat  = 14
                self.cicles = 0
                self.inici_gir(-90 * self._dir_esq_original)
            elif (dist >= 0.30 and d_front > 0.55) or dist >= 0.70:
                if dist >= 0.70:
                    self.get_logger().warn('Estat 13: fallback distància màxima assolit')
                else:
                    self.get_logger().info(f'Frontal lliure ({d_front:.2f} m) → tornant ruta')
                self._cicles_obstacle_frontal = 0
                self.estat  = 14
                self.cicles = 0
                self.inici_gir(-90 * self._dir_esq_original)
            else:
                cmd.twist.linear.x = 0.2

        elif self.estat == 14:
            if self.gir_acabat():
                self.estat  = 15
                self.cicles = 0
                self._cicles_obstacle_frontal = 0
                self.inici_recta()
            else:
                cmd.twist.angular.z = -0.5 * self._dir_esq_original

        elif self.estat == 15:
            graus_orig = 90 if self._dir_esq_original == 1 else 270
            d_orig     = self.distancia_con(graus_orig, finestra=20)
            dist       = self.distancia_recorreguda()
            # FIX #3: obstacle frontal confirmat
            # FIX #2: fallback de distància màxima per objectes grans
            if self.obstacle_frontal_confirmat(0.20) and dist >= 0.15:
                self.get_logger().warn('Obstacle frontal confirmat durant retorn → forçant redreçament')
                self._cicles_obstacle_frontal = 0
                self.estat  = 16
                self.cicles = 0
                self.inici_gir(90 * self._dir_esq_original)
            elif (dist >= 0.25 and d_orig > 0.35) or dist >= 0.60:
                if dist >= 0.60:
                    self.get_logger().warn('Estat 15: fallback distància màxima assolit')
                else:
                    self.get_logger().info('Ruta recuperada → redreçant')
                self._cicles_obstacle_frontal = 0
                self.estat  = 16
                self.cicles = 0
                self.inici_gir(90 * self._dir_esq_original)
            else:
                cmd.twist.linear.x = 0.2

        elif self.estat == 16:
            if self.gir_acabat():
                self.estat          = 0
                self.cicles         = 0
                self.tipus_obstacle = None
                self._cicles_desde_maniobra   = 0
                self._cicles_obstacle_frontal = 0
            else:
                cmd.twist.angular.z = 0.5 * self._dir_esq_original

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MovimentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Aturant node de moviment...')
        node._publicar_aturada()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
