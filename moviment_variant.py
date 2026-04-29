#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32, String, Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math

def angle_diff(a, b):
    """Diferència d'angles en [-pi, pi]"""
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

        # Subscripcions
        self.sub_laser     = self.create_subscription(LaserScan, '/scan', self.laser_callback, qos_sensor)
        self.sub_odom      = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_comptador = self.create_subscription(Int32, '/comptador_objectes', self.comptador_callback, 10)
        self.sub_tipus     = self.create_subscription(String, '/tipus_obstacle', self.tipus_callback, 10)
        self.timer         = self.create_timer(0.1, self.control_callback)

        # Publicadors
        self.pub          = self.create_publisher(TwistStamped, '/cmd_vel', qos_moviment)
        self.pub_maniobra = self.create_publisher(Bool, '/en_maniobra', 10)

        self.get_logger().info('Node de moviment actiu...')

        # Variables d'estat
        self.estat             = 0
        self.cicles            = 0
        self.direccio_s        = 1      # 1=esquerra, -1=dreta
        self.objectes          = 0
        self.tipus_obstacle    = None
        self.direccio_esquivar = 1
        self.laser_ranges      = []
        self.laser_range_min   = 0.1
        self.laser_range_max   = 10.0

        # Odometria per a girs
        self.yaw_actual   = 0.0
        self.yaw_objectiu = None

        # CORRECCIÓ #10: aturat_emergencia s'inicialitza a False i es reinicia correctament
        self.aturat_emergencia = False

        # CORRECCIÓ #9 (debounce integrat al moviment): mínim de cicles entre deteccions
        self._cicles_desde_maniobra = 0

    # ── Callbacks ───────────────────────────────────────────────────────────

    def odom_callback(self, msg):
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        self.yaw_actual = math.atan2(siny_cosp, cosy_cosp)

    def laser_callback(self, msg):
        self.laser_ranges    = msg.ranges
        self.laser_range_min = msg.range_min
        self.laser_range_max = msg.range_max

    def comptador_callback(self, msg):
        self.objectes = msg.data
        if self.objectes >= 5:
            # CORRECCIÓ #3: aturem el robot de forma neta publicant
            # l'estat final abans del proper control_callback
            self.estat = None
            self.get_logger().info('Objectiu complert: 5 objectes trobats!')

    def tipus_callback(self, msg):
        # CORRECCIÓ #9: ignorar deteccions noves si ja estem en maniobra
        # o si acabem de sortir d'una maniobra (debounce de 5 cicles = 0.5 s)
        if self.estat not in (0, None):
            return
        if self._cicles_desde_maniobra < 5:
            return
        self.tipus_obstacle = msg.data

    # ── Helpers ─────────────────────────────────────────────────────────────

    def inici_gir(self, graus):
        """Calcula el yaw objectiu per a un gir de 'graus' (+ = esquerra)."""
        self.yaw_objectiu = self.yaw_actual + math.radians(graus)
        while self.yaw_objectiu >  math.pi: self.yaw_objectiu -= 2 * math.pi
        while self.yaw_objectiu < -math.pi: self.yaw_objectiu += 2 * math.pi

    def gir_acabat(self, tolerancia_deg=2.0):
        """Retorna True quan hem arribat al yaw objectiu."""
        if self.yaw_objectiu is None:
            return False
        return abs(angle_diff(self.yaw_actual, self.yaw_objectiu)) < math.radians(tolerancia_deg)

    def frontal_lliure(self, llindar=0.55):
        # CORRECCIÓ #2: llindar augmentat de 0.25 m → 0.55 m per donar marge de frenada
        # CORRECCIÓ #1/#6: el LiDAR va de 0 a 360 punts,
        # però usem la longitud real per calcular els índexs
        if len(self.laser_ranges) == 0:
            return True
        n = len(self.laser_ranges)
        # Con frontal: ±30° → índexs dinàmics
        graus_con = 30
        idx_con   = int(graus_con * n / 360)
        con = list(self.laser_ranges[0:idx_con]) + list(self.laser_ranges[n - idx_con:])
        valids = [d for d in con if self.laser_range_min < d < self.laser_range_max]
        return min(valids) > llindar if valids else True

    # ── Control principal ────────────────────────────────────────────────────

    def control_callback(self):
        cmd = TwistStamped()
        cmd.header.stamp    = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        # Flag per a detecció — True durant qualsevol maniobra
        en_maniobra = self.estat not in (0, None)
        flag        = Bool()
        flag.data   = en_maniobra
        self.pub_maniobra.publish(flag)

        # Comptador debounce: incrementar quan estem a estat 0
        if self.estat == 0:
            self._cicles_desde_maniobra += 1
        else:
            self._cicles_desde_maniobra = 0

        # ── ESTAT FINAL ──
        if self.estat is None:
            cmd.twist.linear.x  = 0.0
            cmd.twist.angular.z = 0.0
            self.pub.publish(cmd)
            return

        # ── FRE D'EMERGÈNCIA ──
        # CORRECCIÓ #1: s'aplica a TOTS els estats, inclòs l'estat 0
        if not self.frontal_lliure():
            self.aturat_emergencia = True
        else:
            # CORRECCIÓ #10: reiniciem el flag quan el camí és lliure
            self.aturat_emergencia = False

        if self.aturat_emergencia:
            cmd.twist.linear.x  = 0.0
            cmd.twist.angular.z = 0.0
            self.get_logger().warn('EMERGÈNCIA: obstacle frontal!')
            self.pub.publish(cmd)
            return

        # ── ESTAT 0: EXPLORAR ──
        if self.estat == 0:
            if self.tipus_obstacle is None:
                cmd.twist.linear.x  = 0.2
                cmd.twist.angular.z = 0.0
            else:
                cmd.twist.linear.x  = 0.0
                cmd.twist.angular.z = 0.0

                if self.tipus_obstacle == 'PARET':
                    self.get_logger().warn('PARET → Alineant i fent S')
                    self.estat  = 4
                    self.cicles = 0
                else:
                    self.get_logger().warn('OBJECTE → Esquivant')

                    # CORRECCIÓ #6: índexs dinàmics per al con lateral
                    if len(self.laser_ranges) >= 360:
                        n        = len(self.laser_ranges)
                        idx_60   = int(60 * n / 360)
                        idx_300  = int(300 * n / 360)
                        dreta    = self.laser_ranges[idx_300:]
                        esquerra = self.laser_ranges[:idx_60]
                        num_dre  = len([d for d in dreta    if 0.1 < d < 6])
                        num_esq  = len([d for d in esquerra if 0.1 < d < 6])
                        self.direccio_esquivar = -1 if num_esq > num_dre else 1
                        self.get_logger().info(
                            f'ESQ={num_esq} DRE={num_dre} → '
                            f'{"ESQUERRA" if self.direccio_esquivar == -1 else "DRETA"}'
                        )

                    self.inici_gir(90 * self.direccio_esquivar)
                    self.estat  = 10
                    self.cicles = 0

                self.tipus_obstacle = None

        # ── ESTAT 4: ALINEACIÓ PARAL·LELA A LA PARET ──
        elif self.estat == 4:
            if len(self.laser_ranges) >= 360:
                # CORRECCIÓ #5: índexs calculats dinàmicament per a 45° i 315°
                n       = len(self.laser_ranges)
                idx_esq = int(45  * n / 360)
                idx_dre = int(315 * n / 360)
                d_esq   = self.laser_ranges[idx_esq]
                d_dre   = self.laser_ranges[idx_dre]

                # Ignorar lectures invàlides
                if not (self.laser_range_min < d_esq < self.laser_range_max) or \
                   not (self.laser_range_min < d_dre < self.laser_range_max):
                    # Lectures invàlides → passar directament a la maniobra S
                    self.estat = 1
                    self.inici_gir(90 * self.direccio_s)
                else:
                    diff = d_esq - d_dre
                    if abs(diff) < 0.05:
                        self.get_logger().info('Alineat → Iniciant S')
                        self.estat = 1
                        self.inici_gir(90 * self.direccio_s)
                    elif diff > 0:
                        cmd.twist.angular.z =  0.2
                    else:
                        cmd.twist.angular.z = -0.2
            else:
                self.estat = 1
                self.inici_gir(90 * self.direccio_s)

        # ── MANIOBRA EN "S" ──

        elif self.estat == 1:   # Primer gir 90°
            if self.gir_acabat():
                self.estat  = 2
                self.cicles = 0
            else:
                cmd.twist.angular.z = 0.5 * self.direccio_s

        elif self.estat == 2:   # Recta de canvi de fila
            self.cicles += 1
            if self.cicles < 20:
                cmd.twist.linear.x = 0.15
            else:
                self.estat  = 3
                self.cicles = 0
                self.inici_gir(90 * self.direccio_s)

        elif self.estat == 3:   # Segon gir 90°
            if self.gir_acabat():
                self.direccio_s    *= -1
                self.estat          = 0
                self.cicles         = 0
                self.tipus_obstacle = None
                # CORRECCIÓ #9: reiniciem debounce en acabar maniobra S
                self._cicles_desde_maniobra = 0
            else:
                cmd.twist.angular.z = 0.5 * self.direccio_s

        # ── ESQUIVAR OBJECTE ──

        elif self.estat == 10:  # Gir 90° cap al costat lliure
            if self.gir_acabat():
                self.estat  = 11
                self.cicles = 0
            else:
                cmd.twist.angular.z = 0.5 * self.direccio_esquivar

        elif self.estat == 11:  # Avançar lateral
            self.cicles += 1
            if self.cicles < 12:
                cmd.twist.linear.x = 0.2
            else:
                self.estat  = 12
                self.cicles = 0
                self.inici_gir(-90 * self.direccio_esquivar)

        elif self.estat == 12:  # Gir 90° contrari
            if self.gir_acabat():
                self.estat  = 13
                self.cicles = 0
            else:
                cmd.twist.angular.z = -0.5 * self.direccio_esquivar

        elif self.estat == 13:  # Avançar superant objecte
            self.cicles += 1
            if self.cicles < 25:
                cmd.twist.linear.x = 0.2
            else:
                self.estat  = 14
                self.cicles = 0
                self.inici_gir(-90 * self.direccio_esquivar)

        elif self.estat == 14:  # Gir 90° contrari
            if self.gir_acabat():
                self.estat  = 15
                self.cicles = 0
            else:
                cmd.twist.angular.z = -0.5 * self.direccio_esquivar

        elif self.estat == 15:  # Avançar tornant a la ruta
            self.cicles += 1
            if self.cicles < 12:
                cmd.twist.linear.x = 0.2
            else:
                self.estat  = 16
                self.cicles = 0
                self.inici_gir(90 * self.direccio_esquivar)

        elif self.estat == 16:  # Gir 90° redreçant
            if self.gir_acabat():
                self.estat          = 0
                self.cicles         = 0
                self.tipus_obstacle = None
                # CORRECCIÓ #9: reiniciem debounce en acabar esquiva
                self._cicles_desde_maniobra = 0
            else:
                cmd.twist.angular.z = 0.5 * self.direccio_esquivar

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MovimentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stop = TwistStamped()
        stop.header.stamp    = node.get_clock().now().to_msg()
        stop.twist.linear.x  = 0.0
        stop.twist.angular.z = 0.0
        node.pub.publish(stop)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
