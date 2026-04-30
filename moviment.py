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
    """Diferència d'angles normalitzada a [-pi, pi]."""
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

        # ── Variables d'estat ────────────────────────────────────────────
        self.estat             = 0       # estat actual de la màquina d'estats
        self.cicles            = 0       # comptador de cicles dins d'un estat
        self.objectes          = 0       # objectes trobats fins ara
        self.tipus_obstacle    = None    # 'PARET' | 'OBJECTE' | None
        self.direccio_s        = 1       # sentit de la maniobra S: 1=esquerra, -1=dreta
        self.direccio_esquivar = 1       # sentit d'esquiva de l'objecte

        # LiDAR
        self.laser_ranges    = []
        self.laser_range_min = 0.1
        self.laser_range_max = 10.0

        # Odometria per a girs i control de distància
        self.yaw_actual    = 0.0
        self.yaw_objectiu  = None
        self.robot_x       = 0.0
        self.robot_y       = 0.0
        self.x_inici_recta = 0.0   # posició en iniciar un tram recte
        self.y_inici_recta = 0.0

        # Fre d'emergència
        self.aturat_emergencia = False

        # Debounce per evitar redeteccions just en sortir d'una maniobra
        self._cicles_desde_maniobra = 0

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

    def tipus_callback(self, msg):
        # Ignorar si ja estem en maniobra o no ha passat el debounce
        if self.estat not in (0, None):
            return
        if self._cicles_desde_maniobra < 5:
            return
        self.tipus_obstacle = msg.data

    # ── Helpers ──────────────────────────────────────────────────────────

    def inici_recta(self):
        """Desa la posició actual com a origen d'un tram recte."""
        self.x_inici_recta = self.robot_x
        self.y_inici_recta = self.robot_y

    def distancia_recorreguda(self):
        """Distància euclidiana des de l'inici del tram recte actual."""
        return math.sqrt(
            (self.robot_x - self.x_inici_recta) ** 2 +
            (self.robot_y - self.y_inici_recta) ** 2
        )

    def inici_gir(self, graus):
        """Estableix el yaw objectiu per a un gir de 'graus' (+ = esquerra)."""
        self.yaw_objectiu = self.yaw_actual + math.radians(graus)
        while self.yaw_objectiu >  math.pi: self.yaw_objectiu -= 2 * math.pi
        while self.yaw_objectiu < -math.pi: self.yaw_objectiu += 2 * math.pi

    def gir_acabat(self, tolerancia_deg=2.0):
        """Retorna True quan hem arribat al yaw objectiu (dins la tolerància)."""
        if self.yaw_objectiu is None:
            return False
        return abs(angle_diff(self.yaw_actual, self.yaw_objectiu)) < math.radians(tolerancia_deg)

    def frontal_lliure(self, llindar=0.55):
        """Retorna True si el con frontal (±45°) no té cap obstacle a menys de llindar metres.
        S'usa ±45° per ser coherent amb el con de detecció de deteccio.py (±60°) i
        reaccionar abans que el robot s'hi acosti massa."""
        if not self.laser_ranges:
            return True
        n = len(self.laser_ranges)
        idx = int(45 * n / 360)
        con = list(self.laser_ranges[:idx]) + list(self.laser_ranges[n - idx:])
        valids = [d for d in con if self.laser_range_min < d < self.laser_range_max]
        return (min(valids) > llindar) if valids else True

    def _publicar_aturada(self):
        """Publica velocitat zero."""
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

        # Publicar flag de maniobra per pausar detecció
        en_maniobra = self.estat not in (0, None)
        flag = Bool()
        flag.data = en_maniobra
        self.pub_maniobra.publish(flag)

        # Debounce: incrementar comptador quan estem explorant lliurement
        if self.estat == 0:
            self._cicles_desde_maniobra += 1
        else:
            self._cicles_desde_maniobra = 0

        # ── ESTAT FINAL: robot parat ──
        if self.estat is None:
            self._publicar_aturada()
            return

        # ── FRE D'EMERGÈNCIA (s'aplica a tots els estats) ──
        if not self.frontal_lliure():
            self.aturat_emergencia = True
        else:
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
                    self.get_logger().warn('PARET → Alineant i fent maniobra S')
                    self.estat  = 4
                    self.cicles = 0

                else:  # OBJECTE
                    self.get_logger().warn('OBJECTE → Esquivant')

                    # Decidir direcció d'esquiva: el costat amb més raigs lliures
                    if len(self.laser_ranges) >= 360:
                        n       = len(self.laser_ranges)
                        idx_60  = int(60  * n / 360)
                        idx_300 = int(300 * n / 360)
                        esq = [d for d in self.laser_ranges[:idx_60]    if 0.1 < d < 6.0]
                        dre = [d for d in self.laser_ranges[idx_300:]   if 0.1 < d < 6.0]
                        num_esq = len(esq)
                        num_dre = len(dre)
                        # Preferim el costat amb menys punts (= més espai lliure)
                        self.direccio_esquivar = -1 if num_esq < num_dre else 1
                        self.get_logger().info(
                            f'ESQ={num_esq} DRE={num_dre} → '
                            f'{"ESQUERRA" if self.direccio_esquivar == -1 else "DRETA"}'
                        )
                    else:
                        # LiDAR no llest: esquivar per defecte a l'esquerra
                        self.direccio_esquivar = 1

                    self.inici_gir(90 * self.direccio_esquivar)
                    self.estat  = 10
                    self.cicles = 0

                self.tipus_obstacle = None

        # ── ESTAT 4: ALINEACIÓ PARAL·LELA A LA PARET ──
        elif self.estat == 4:
            if len(self.laser_ranges) >= 360:
                n       = len(self.laser_ranges)
                idx_esq = int(45  * n / 360)
                idx_dre = int(315 * n / 360)
                d_esq   = self.laser_ranges[idx_esq]
                d_dre   = self.laser_ranges[idx_dre]

                valids = (
                    self.laser_range_min < d_esq < self.laser_range_max and
                    self.laser_range_min < d_dre < self.laser_range_max
                )
                if not valids:
                    # Lectures invàlides → passar directament a la maniobra S
                    self.estat = 1
                    self.inici_gir(90 * self.direccio_s)
                else:
                    diff = d_esq - d_dre
                    if abs(diff) < 0.05:
                        self.get_logger().info('Alineat → Iniciant maniobra S')
                        self.estat = 1
                        self.inici_gir(90 * self.direccio_s)
                    elif diff > 0:
                        cmd.twist.angular.z =  0.2
                    else:
                        cmd.twist.angular.z = -0.2
            else:
                self.estat = 1
                self.inici_gir(90 * self.direccio_s)

        # ── MANIOBRA EN "S" ──────────────────────────────────────────────

        elif self.estat == 1:   # Primer gir 90°
            if self.gir_acabat():
                self.estat  = 2
                self.cicles = 0
            else:
                cmd.twist.angular.z = 0.5 * self.direccio_s

        elif self.estat == 2:   # Recta de canvi de fila
            if self.cicles == 0:
                self.inici_recta()
            self.cicles += 1
            if self.distancia_recorreguda() < 0.30:
                cmd.twist.linear.x = 0.15
            else:
                self.estat  = 3
                self.cicles = 0
                self.inici_gir(90 * self.direccio_s)

        elif self.estat == 3:   # Segon gir 90°
            if self.gir_acabat():
                self.direccio_s    *= -1   # invertir sentit per al pròxim passada
                self.estat          = 0
                self.cicles         = 0
                self.tipus_obstacle = None
                self._cicles_desde_maniobra = 0
            else:
                cmd.twist.angular.z = 0.5 * self.direccio_s

        # ── ESQUIVAR OBJECTE ─────────────────────────────────────────────

        elif self.estat == 10:  # Gir 90° cap al costat lliure
            if self.gir_acabat():
                self.estat  = 11
                self.cicles = 0
            else:
                cmd.twist.angular.z = 0.5 * self.direccio_esquivar

        elif self.estat == 11:  # Avançar lateral (saltar l'amplada de l'objecte)
            if self.cicles == 0:
                self.inici_recta()
            self.cicles += 1
            if self.distancia_recorreguda() < 0.35:
                cmd.twist.linear.x = 0.2
            else:
                self.estat  = 12
                self.cicles = 0
                self.inici_gir(-90 * self.direccio_esquivar)

        elif self.estat == 12:  # Gir 90° contrari (encarar l'objecte de costat)
            if self.gir_acabat():
                self.estat  = 13
                self.cicles = 0
            else:
                cmd.twist.angular.z = -0.5 * self.direccio_esquivar

        elif self.estat == 13:  # Avançar superant l'objecte en profunditat
            if self.cicles == 0:
                self.inici_recta()
            self.cicles += 1
            if self.distancia_recorreguda() < 0.55:
                cmd.twist.linear.x = 0.2
            else:
                self.estat  = 14
                self.cicles = 0
                self.inici_gir(-90 * self.direccio_esquivar)

        elif self.estat == 14:  # Gir 90° contrari (tornar a encarar la direcció original)
            if self.gir_acabat():
                self.estat  = 15
                self.cicles = 0
            else:
                cmd.twist.angular.z = -0.5 * self.direccio_esquivar

        elif self.estat == 15:  # Avançar tornant a la ruta original
            if self.cicles == 0:
                self.inici_recta()
            self.cicles += 1
            if self.distancia_recorreguda() < 0.35:
                cmd.twist.linear.x = 0.2
            else:
                self.estat  = 16
                self.cicles = 0
                self.inici_gir(90 * self.direccio_esquivar)

        elif self.estat == 16:  # Gir final redreçant el robot
            if self.gir_acabat():
                self.estat          = 0
                self.cicles         = 0
                self.tipus_obstacle = None
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
        node.get_logger().info('Aturant node de moviment...')
        node._publicar_aturada()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
