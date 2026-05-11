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

        self.sub_laser     = self.create_subscription(LaserScan, '/scan', self.laser_callback, qos_sensor)
        self.sub_odom      = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_comptador = self.create_subscription(Int32, '/comptador_objectes', self.comptador_callback, 10)
        self.sub_tipus     = self.create_subscription(String, '/tipus_obstacle', self.tipus_callback, 10)
        self.timer         = self.create_timer(0.1, self.control_callback)

        self.pub          = self.create_publisher(TwistStamped, '/cmd_vel', qos_moviment)
        self.pub_maniobra = self.create_publisher(Int32, '/en_maniobra', 10)

        # ── Estat ───────────────────────────────────────────────────────
        self.estat             = 0
        self.estat_loguejat    = None
        self.cicles            = 0
        self.objectes          = 0
        self.tipus_obstacle    = None
        self.direccio_esquivar = -1
        self.direccio_paret    = -1

        # LiDAR
        self.laser_ranges    = []
        self.laser_range_min = 0.1
        self.laser_range_max = 10.0

        # Girs per yaw absolut
        self.yaw_actual   = 0.0
        self.yaw_objectiu = None

        # Odometria per a rectes
        self.robot_x       = 0.0
        self.robot_y       = 0.0
        self.x_inici_recta = 0.0
        self.y_inici_recta = 0.0

        # Esperar odometria real abans de moure's
        self.odom_rebuda = False

        self.noms_estat = {
            None: 'FINAL',
            0:  'EXPLORAR',
            1:  'PARET - Gir 70° alineament',
            2:  'PARET - Gir 90° costat lliure',
            10: 'ESQUIVA - Gir 90° costat lliure',
            11: 'ESQUIVA - Avançar lateral',
            12: 'ESQUIVA - Gir 90° contrari',
            13: 'ESQUIVA - Avançar superar objecte',
            14: 'ESQUIVA - Gir 90° contrari (2)',
            15: 'ESQUIVA - Avançar tornar ruta',
            16: 'ESQUIVA - Gir 90° redreçar',
        }

        self.get_logger().info('Node de moviment actiu...')

    # ── Logging ──────────────────────────────────────────────────────────

    def log_estat(self):
        if self.estat != self.estat_loguejat:
            nom = self.noms_estat.get(self.estat, f'DESCONEGUT ({self.estat})')
            self.get_logger().info(f'[ESTAT {self.estat}] {nom}')
            self.estat_loguejat = self.estat

    # ── Callbacks ────────────────────────────────────────────────────────

    def laser_callback(self, msg):
        self.laser_ranges    = msg.ranges
        self.laser_range_min = msg.range_min
        self.laser_range_max = msg.range_max

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw_actual = math.atan2(siny, cosy)
        if not self.odom_rebuda:
            self.odom_rebuda = True
            self.get_logger().info(f'Odometria inicial: yaw={math.degrees(self.yaw_actual):.1f}°')

    def comptador_callback(self, msg):
        self.objectes = msg.data
        if self.objectes >= 5:
            self.estat = None
            self.get_logger().info('Objectiu complert: 5 objectes trobats!')
            self._publicar_aturada()

    def tipus_callback(self, msg):
        # Durant girs la detecció està OFF — ignorar per seguretat
        if self.estat in (1, 2, 10, 12, 14, 16):
            return
        self.get_logger().info(f"--- REBUT: '{msg.data}' ---")
        self.tipus_obstacle = msg.data

    # ── Helpers de gir ───────────────────────────────────────────────────

    def inici_gir(self, graus):
        """Yaw objectiu absolut. Gestiona wrap-around."""
        self.yaw_objectiu = self.yaw_actual + math.radians(graus)
        while self.yaw_objectiu >  math.pi: self.yaw_objectiu -= 2 * math.pi
        while self.yaw_objectiu < -math.pi: self.yaw_objectiu += 2 * math.pi
        self.cicles = 0

    def gir_acabat(self, tolerancia_deg=4.0):
        if self.yaw_objectiu is None:
            return True
        return abs(angle_diff(self.yaw_actual, self.yaw_objectiu)) < math.radians(tolerancia_deg)

    def vel_gir(self, signe):
        """Velocitat proporcional a l'error: frena als últims 20°."""
        if self.yaw_objectiu is None:
            return 0.0
        error = abs(angle_diff(self.yaw_actual, self.yaw_objectiu))
        vel = min(0.35, max(0.08, error * (0.35 / math.radians(20))))
        return signe * vel

    # ── Helpers de recta ─────────────────────────────────────────────────

    def inici_recta(self):
        self.x_inici_recta = self.robot_x
        self.y_inici_recta = self.robot_y
        self.cicles        = 0

    def distancia_recorreguda(self):
        return math.sqrt(
            (self.robot_x - self.x_inici_recta) ** 2 +
            (self.robot_y - self.y_inici_recta) ** 2
        )

    # ── Helpers de LiDAR ─────────────────────────────────────────────────

    def distancia_con(self, graus_centre, finestra=15):
        """Distància mínima en un con de ±finestra° (índexs dinàmics)."""
        if not self.laser_ranges:
            return 999.0
        n     = len(self.laser_ranges)
        idx_c = int(graus_centre * n / 360) % n
        idx_f = max(1, int(finestra * n / 360))
        idxs  = [(idx_c + i) % n for i in range(-idx_f, idx_f + 1)]
        vals  = [self.laser_ranges[i] for i in idxs
                 if self.laser_range_min < self.laser_ranges[i] < self.laser_range_max]
        return min(vals) if vals else 999.0

    def calcular_costat_lliure(self):
        """Índexs dinàmics. Mira diagonal-frontal 30°-90° de cada costat."""
        if not self.laser_ranges:
            return -1
        d_esq = self.distancia_con(60,  finestra=30)
        d_dre = self.distancia_con(300, finestra=30)
        costat = 1 if d_esq > d_dre else -1
        self.get_logger().info(
            f'Costat lliure: ESQ={d_esq:.2f} DRE={d_dre:.2f} → {"ESQ" if costat==1 else "DRE"}'
        )
        return costat

    def _publicar_aturada(self):
        cmd = TwistStamped()
        cmd.header.stamp    = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x  = 0.0
        cmd.twist.angular.z = 0.0
        self.pub.publish(cmd)

    # ── Gestió d'obstacles ───────────────────────────────────────────────

    def comprovar_obstacle(self, estat_seguent):
        """Decideix l'estat següent en acabar un gir o una recta."""
        # Netejar espais residuals per evitar falsos positius
        text          = str(self.tipus_obstacle).strip()
        hi_ha_paret   = 'PARET'   in text
        hi_ha_objecte = 'OBJECTE' in text

        self.get_logger().info(f'[COMPROVAR] Tipus: "{text}" | Estat actual: {self.estat}')

        if hi_ha_paret:
            self.get_logger().warn('PARET → Alineant')
            self.direccio_paret = self.calcular_costat_lliure()
            self.tipus_obstacle = None
            self.estat          = 1
            self.inici_gir(70 * self.direccio_paret)

        elif hi_ha_objecte:
            if self.estat >= 10:
                # Ja esquivant: ignorar i continuar al següent estat previst
                self.get_logger().info(f'Ignorant objecte durant maniobra (estat {self.estat})')
                self.tipus_obstacle = None
                self._transicio(estat_seguent)
            else:
                self.get_logger().warn('OBJECTE → Esquivant')
                self.direccio_esquivar = self.calcular_costat_lliure()
                self.tipus_obstacle    = None
                self.estat             = 10
                self.inici_gir(90 * self.direccio_esquivar)

        else:
            # Cap obstacle: anar directament a l'estat següent
            self.tipus_obstacle = None
            self._transicio(estat_seguent)

    def _transicio(self, estat_seguent):
        """Fa la transició a l'estat indicat iniciant el gir o recta corresponent."""
        self.estat  = estat_seguent
        self.cicles = 0
        graus_gir = {
            1:   70 * self.direccio_paret,
            2:   90 * self.direccio_paret,
            10:  90 * self.direccio_esquivar,
            12: -90 * self.direccio_esquivar,
            14: -90 * self.direccio_esquivar,
            16:  90 * self.direccio_esquivar,
        }
        if estat_seguent in graus_gir:
            self.inici_gir(graus_gir[estat_seguent])
        else:
            self.inici_recta()

    # ── Control principal (10 Hz) ─────────────────────────────────────────

    def control_callback(self):
        cmd = TwistStamped()
        cmd.header.stamp    = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        self.log_estat()

        # Flag maniobra per a deteccio.py
        if self.estat in (1, 2, 10, 12, 14, 16):
            estat_maniobra = 1   # girant → detecció OFF
        elif self.estat in (11, 13, 15):
            estat_maniobra = 2   # recta esquiva → llindar reduït
        else:
            estat_maniobra = 0   # explorant → detecció normal
        flag      = Int32()
        flag.data = estat_maniobra
        self.pub_maniobra.publish(flag)

        # ── ESTAT FINAL ──
        if self.estat is None:
            self._publicar_aturada()
            return

        # ── ESTAT 0: EXPLORAR ────────────────────────────────────────────
        if self.estat == 0:
            if not self.odom_rebuda:
                # Esperar odometria real abans de moure's
                self.pub.publish(cmd)
                return
            if self.tipus_obstacle is None:
                cmd.twist.linear.x  = 0.2
                cmd.twist.angular.z = 0.0
                # sense correccio_deriva — causava gir inicial espuri
            else:
                cmd.twist.linear.x  = 0.0
                cmd.twist.angular.z = 0.0
                self.comprovar_obstacle(0)

        # ── PARET: gir 70° alineament ────────────────────────────────────
        elif self.estat == 1:
            if self.gir_acabat():
                self.direccio_paret = self.calcular_costat_lliure()
                self.estat = 2
                self.inici_gir(90 * self.direccio_paret)
            else:
                cmd.twist.angular.z = self.vel_gir(self.direccio_paret)

        # ── PARET: gir 90° cap al costat lliure ──────────────────────────
        elif self.estat == 2:
            if self.gir_acabat():
                self.comprovar_obstacle(0)
            else:
                cmd.twist.angular.z = self.vel_gir(self.direccio_paret)

        # ── ESQUIVA: gir 90° cap al costat lliure ────────────────────────
        elif self.estat == 10:
            if self.gir_acabat():
                self.comprovar_obstacle(11)
            else:
                cmd.twist.angular.z = self.vel_gir(self.direccio_esquivar)

        # ── ESQUIVA: avançar lateral ──────────────────────────────────────
        # CANVI: 12 cicles → 18 cicles (0.2m/s × 1.8s = 36cm, prou per superar un objecte)
        elif self.estat == 11:
            self.cicles += 1
            if self.tipus_obstacle is not None:
                self.comprovar_obstacle(12)
            elif self.cicles < 18:
                cmd.twist.linear.x = 0.2
            else:
                self.comprovar_obstacle(12)

        # ── ESQUIVA: gir 90° contrari ─────────────────────────────────────
        elif self.estat == 12:
            if self.gir_acabat():
                self.comprovar_obstacle(13)
            else:
                cmd.twist.angular.z = self.vel_gir(-self.direccio_esquivar)

        # ── ESQUIVA: avançar superar objecte ──────────────────────────────
        # CANVI: 25 cicles → 35 cicles (0.2m/s × 3.5s = 70cm)
        elif self.estat == 13:
            self.cicles += 1
            if self.tipus_obstacle is not None:
                self.comprovar_obstacle(14)
            elif self.cicles < 35:
                cmd.twist.linear.x = 0.2
            else:
                self.comprovar_obstacle(14)

        # ── ESQUIVA: gir 90° contrari (2) ─────────────────────────────────
        elif self.estat == 14:
            if self.gir_acabat():
                self.comprovar_obstacle(15)
            else:
                cmd.twist.angular.z = self.vel_gir(-self.direccio_esquivar)

        # ── ESQUIVA: avançar tornar a ruta ────────────────────────────────
        # CANVI: 12 cicles → 18 cicles (simètric a l'estat 11)
        elif self.estat == 15:
            self.cicles += 1
            if self.tipus_obstacle is not None:
                self.comprovar_obstacle(16)
            elif self.cicles < 18:
                cmd.twist.linear.x = 0.2
            else:
                self.comprovar_obstacle(16)

        # ── ESQUIVA: gir 90° redreçar ─────────────────────────────────────
        elif self.estat == 16:
            if self.gir_acabat():
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
        node.get_logger().info('Aturant node de moviment...')
        node._publicar_aturada()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
