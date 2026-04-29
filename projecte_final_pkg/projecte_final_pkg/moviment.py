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

class MovimentNode(Node):
    def __init__(self):
        super().__init__('controlador_moviment')

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS per al moviment
        qos_moviment = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscripcions
        self.sub_laser = self.create_subscription(LaserScan, '/scan', self.laser_callback, qos_sensor)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_comptador = self.create_subscription(Int32, '/comptador_objectes', self.comptador_callback, 10)
        self.sub_tipus = self.create_subscription(String, '/tipus_obstacle', self.tipus_callback, 10)
        self.timer = self.create_timer(0.1, self.control_callback)

        # Publicador
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', qos_moviment)
        self.pub_maniobra = self.create_publisher(Bool, '/en_maniobra', 10)  # flag per a deteccio
        self.get_logger().info('Node de moviment actiu...')

        #  Variables per saber estat
        self.estat = 0
        self.cicles = 0
        self.objectes = 0
        self.tipus_obstacle = None
        self.direccio_esquivar = 1  # 1=esquerra, -1=dreta (decidit pel laser)
        self.direccio_paret = 1     # 1=esquerra, -1=dreta (decidit pel laser)
        self.laser_ranges = []      # última lectura del làser

        # Odometria per als girs
        self.angle_actual = 0.0     # yaw actual del robot
        self.angle_inici_gir = None # yaw quan va començar el gir

    # Callbacks
    def laser_callback(self, msg):
        self.laser_ranges = msg.ranges  # guardem per usar al control_callback

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.angle_actual = math.atan2(siny, cosy)

    def comptador_callback(self, msg):
        self.objectes = msg.data
        if self.objectes >= 5:
            self.estat = None
            self.get_logger().info('Objectiu complert: 5 objectes trobats!')

    def tipus_callback(self, msg):
        self.tipus_obstacle = msg.data  # 'PARET' o 'OBJECTE'

    def iniciar_gir(self):
        """Guardar angle inicial quan comença un gir"""
        self.angle_inici_gir = self.angle_actual

    def angle_girat(self):
        """Retorna quants radians hem girat des de angle_inici_gir (sempre positiu)"""
        if self.angle_inici_gir is None:
            return 0.0
        diff = self.angle_actual - self.angle_inici_gir
        # Normalitzar a [-pi, pi]
        diff = (diff + math.pi) % (2 * math.pi) - math.pi
        return abs(diff)

    def calcular_costat_lliure(self):
        """Decideix cap a quin costat hi ha més espai lliure"""
        if len(self.laser_ranges) >= 360:
            dreta = self.laser_ranges[300:360]
            esquerra = self.laser_ranges[0:60]
            valors_validsdre = [d for d in dreta if 0.1 < d < 6]
            valors_validsesq = [d for d in esquerra if 0.1 < d < 6]
            num_dre = len(valors_validsdre)
            num_esq = len(valors_validsesq)
            if num_esq > num_dre:
                return -1  # més espai a l'esquerra
            else:
                return 1   # més espai a la dreta
        return 1  # per defecte

    def control_callback(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        # Publicar flag en_maniobra durant els girs de l'esquiva
        # durant els avanços la detecció està activa per veure obstacles nous
        flag = Bool()
        flag.data = self.estat in (10, 12, 14, 16)
        self.pub_maniobra.publish(flag)

        #  ESTAT FINAL
        if self.estat is None:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            self.pub.publish(cmd)
            return

        #  EXPLORAR EN LÍNIA RECTA
        if self.estat == 0:
            if self.tipus_obstacle is None:
                cmd.twist.linear.x = 0.2
                cmd.twist.angular.z = 0.0
            else:
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = 0.0

                if self.tipus_obstacle == 'PARET':
                    self.get_logger().warn('PARET detectada → Girant 45° cap al costat lliure')
                    self.direccio_paret = self.calcular_costat_lliure()
                    self.estat = 1
                    self.iniciar_gir()
                else:
                    self.get_logger().warn('OBJECTE detectat → Esquivant')
                    self.direccio_esquivar = self.calcular_costat_lliure()
                    self.estat = 10
                    self.iniciar_gir()
                    self.get_logger().info(
                        f'Gir {"ESQUERRA" if self.direccio_esquivar == 1 else "DRETA"}'
                    )

                self.tipus_obstacle = None
                self.cicles = 0

        #  MANIOBRA PARET - gir de 45° cap al costat lliure
        elif self.estat == 1:
            if self.angle_girat() < math.pi / 4:  # 45°
                cmd.twist.angular.z = 0.5 * self.direccio_paret
            else:
                self.estat = 0
                self.cicles = 0
                self.tipus_obstacle = None

        #  ESQUIVAR OBJECTE (costat decidit pel làser)
        elif self.estat == 10:  # Gir 90° cap al costat lliure
            if self.angle_girat() < math.pi / 2:
                cmd.twist.angular.z = 0.5 * self.direccio_esquivar
            else:
                self.estat = 11
                self.cicles = 0

        elif self.estat == 11:  # Avançar (esquivar lateral) - deteccio pausada
            self.cicles += 1
            if self.cicles < 12:
                cmd.twist.linear.x = 0.2
            else:
                self.estat = 12
                self.cicles = 0
                self.iniciar_gir()

        elif self.estat == 12:  # Gir 90° cap al costat contrari
            if self.angle_girat() < math.pi / 2:
                cmd.twist.angular.z = -0.5 * self.direccio_esquivar
            else:
                self.estat = 13
                self.cicles = 0

        elif self.estat == 13:  # Avançar (superar objecte) - deteccio pausada
            self.cicles += 1
            if self.cicles < 25:
                cmd.twist.linear.x = 0.2
            else:
                self.estat = 14
                self.cicles = 0
                self.iniciar_gir()

        elif self.estat == 14:  # Gir 90° cap al costat contrari
            if self.angle_girat() < math.pi / 2:
                cmd.twist.angular.z = -0.5 * self.direccio_esquivar
            else:
                self.estat = 15
                self.cicles = 0

        elif self.estat == 15:  # Avançar per tornar a la ruta - deteccio pausada
            self.cicles += 1
            if self.cicles < 12:
                cmd.twist.linear.x = 0.2
            else:
                self.estat = 16
                self.cicles = 0
                self.iniciar_gir()

        elif self.estat == 16:  # Gir 90° cap al costat original (redrecem)
            if self.angle_girat() < math.pi / 2:
                cmd.twist.angular.z = 0.5 * self.direccio_esquivar
            else:
                self.estat = 0
                self.cicles = 0
                self.tipus_obstacle = None

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MovimentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stop = TwistStamped()
        stop.header.stamp = node.get_clock().now().to_msg()
        stop.twist.linear.x = 0.0
        stop.twist.angular.z = 0.0
        node.pub.publish(stop)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
