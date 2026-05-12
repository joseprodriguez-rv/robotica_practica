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

        #subscripcions, en aquestes apereixen les subscripcions de base més les relacionades amb els altres dos nodes
        self.sub_laser = self.create_subscription(LaserScan, '/scan', self.laser_callback, qos_sensor)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_comptador = self.create_subscription(Int32, '/comptador_objectes', self.comptador_callback, 10)
        self.sub_tipus = self.create_subscription(String, '/tipus_obstacle', self.tipus_callback, 10)
        self.timer = self.create_timer(0.1, self.control_callback)

        #publicador de velocitat i maniobres per detecció i el robot en si
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', qos_moviment)
        self.pub_maniobra = self.create_publisher(Int32, '/en_maniobra', 10)  # flag per a deteccio
        self.get_logger().info('Node de moviment actiu...')

        #  variables per saber estat i variables de la classe
        self.estat = 0
        self.estat_loguejat = None   # canvia quan hi ha canvis en l'estat
        self.cicles = 0              # cicles maniobres 
        self.objectes = 0            # counter
        self.tipus_obstacle = None   # paret / obstacle
        self.direccio_esquivar = -1  # +1=esquerra, -1=dreta
        self.direccio_paret = -1     # +1=esquerra, -1=dreta
        self.laser_ranges = []       #última lectura del làser

        # Odometria per als girs
        self.angle_actual = 0.0     # yaw actual del robot
        self.angle_inici_gir = None # yaw quan va començar el gir

        self.noms_estat = {
            None: 'FINAL - Objectiu complert',
            0: 'EXPLORAR en linia recta',
            1: 'PARET - Gir 150° cap al costat lliure',
            10: 'ESQUIVA - Gir 90 cap al costat lliure',
            11: 'ESQUIVA - Avancar lateral',
            12: 'ESQUIVA - GIr 90 cap al contrari',
            13: 'ESQUIVA - Avancar superar objecte',
            14: 'ESQUIVA - Gir 90 cap al contrari (2)',
            15: 'ESQUIVA - Avancar tornar a ruta',
            16: 'ESQUIVA - Gir 90 redrecar',
            }

    def log_estat(self):
        if self.estat != self.estat_loguejat:
            nom = self.noms_estat.get(self.estat, f'DESCONEGUT ({self.estat})')
            self.get_logger().info(f'[ESTAT {self.estat}] {nom}')
            self.estat_loguejat = self.estat

    def laser_callback(self, msg):
        self.laser_ranges = msg.ranges  # guardem laser cada cop

    def odom_callback(self, msg):
        # Convertir quaternion -> yaw
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
        # només actualitza si no estem girant
        if self.estat not in (1, 10, 12, 14, 16):
            self.tipus_obstacle = msg.data  # 'PARET' o 'OBJECTE'

    def iniciar_gir(self):
        #guardar angle inicial
        self.angle_inici_gir = self.angle_actual

    def angle_girat(self):
        #per saber quants radians s'ha girat des de l'inici
        if self.angle_inici_gir is None:
            return 0.0
        diff = self.angle_actual - self.angle_inici_gir
        # Normalitzar a [-pi, pi]
        diff = (diff + math.pi) % (2 * math.pi) - math.pi
        return abs(diff)

    def calcular_costat_lliure(self):
        #quin costat hi ha més espai
        #+1 esquerra, -1 dreta 
        self.get_logger().warn(f'[CALCULAR] entrant. len(laser_ranges)={len(self.laser_ranges)}')
        if len(self.laser_ranges) >= 360:
            dreta = self.laser_ranges[270:360]
            esquerra = self.laser_ranges[0:90]
            valors_validsdre = [d for d in dreta if 0.1 < d < 6]
            valors_validsesq = [d for d in esquerra if 0.1 < d < 6]
            num_dre = len(valors_validsdre)
            num_esq = len(valors_validsesq)
            self.get_logger().warn(
                f'[CALCULAR] num_esq={num_esq}  num_dre={num_dre}  '
                f'-> {"+1 (esquerra)" if num_esq > num_dre else "-1 (dreta)"}'
            )
            if num_esq > num_dre:
                return 1   #més espai a l'esquerra -> anar a l'esquerra
            else:
                return -1  #més espai a la dreta -> anar a la dreta
        self.get_logger().warn(f'[CALCULAR] retornant -1 per defecte (len < 360)')
        return -1  #per defecte (dreta)

    def comprovar_obstacle(self, estat_seguent):
        #comprovar si hi ha un obstacle després de cada maniobres
        #evitar xocs
        self.get_logger().info(
            f'[COMPROVAR] tipus_obstacle={self.tipus_obstacle!r}  '
            f'estat_actual={self.estat}  estat_seguent={estat_seguent}  '
            f'len(laser)={len(self.laser_ranges)}'
        )
        if self.tipus_obstacle == 'PARET':
            self.direccio_paret = self.calcular_costat_lliure()
            self.tipus_obstacle = None
            self.estat = 1
            self.iniciar_gir()
        elif self.tipus_obstacle == 'OBJECTE':
            self.direccio_esquivar = self.calcular_costat_lliure()
            self.get_logger().warn(f'OBJECTE detectat -> Esquivant cap a {
                "l\'ESQUERRA" if self.direccio_esquivar == 1 else "la DRETA"}')
            self.tipus_obstacle = None
            self.estat = 10
            self.iniciar_gir()
        else:
            self.estat = estat_seguent
            self.cicles = 0
            if estat_seguent in (10, 12, 14, 16):  #si el seguent és un gir, iniciar
                self.iniciar_gir()

    def control_callback(self):
        #moviment
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        self.log_estat()

        en_maniobra = Int32()
        if self.estat in (1, 10, 12, 14, 16):
            en_maniobra.data = 1   #girant - detecció desactivada
        elif self.estat in (11, 13, 15):
            en_maniobra.data = 2   #avançant en esquiva - llindar reduït
        else:
            en_maniobra.data = 0   # explorant - detecció normal
        self.pub_maniobra.publish(en_maniobra)

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
                self.comprovar_obstacle(0)

        #  MANIOBRA PARET - estat 1: gir de 150° cap al costat lliure
        elif self.estat == 1:
            if self.angle_girat() < 5 * math.pi / 6:  # 150°
                cmd.twist.angular.z = 0.5 * self.direccio_paret
            else:
                # Gir acabat -> comprovar si hi ha obstacle nou
                self.comprovar_obstacle(0)

        #  ESQUIVAR OBJECTE (costat decidit pel làser)
        elif self.estat == 10:  # Gir 90° cap al costat lliure
            if self.angle_girat() < math.pi / 2:
                cmd.twist.angular.z = 0.5 * self.direccio_esquivar
            else:
                # Gir acabat -> comprovar si hi ha obstacle nou
                self.comprovar_obstacle(11)

        elif self.estat == 11:  # Avançar (esquivar lateral) - deteccio activa
            self.cicles += 1
            if self.tipus_obstacle is not None:
                self.comprovar_obstacle(12)
            elif self.cicles < 12:
                cmd.twist.linear.x = 0.2
            else:
                # Avanç acabat -> comprovar si hi ha obstacle nou
                self.comprovar_obstacle(12)

        elif self.estat == 12:  # Gir 90° cap al costat contrari
            if self.angle_girat() < math.pi / 2:
                cmd.twist.angular.z = -0.5 * self.direccio_esquivar
            else:
                # Gir acabat -> comprovar si hi ha obstacle nou
                self.comprovar_obstacle(13)

        elif self.estat == 13:  # Avançar (superar objecte) - deteccio activa
            self.cicles += 1
            if self.tipus_obstacle is not None:
                self.comprovar_obstacle(14)
            elif self.cicles < 25:
                cmd.twist.linear.x = 0.2
            else:
                # Avanç acabat -> comprovar si hi ha obstacle nou
                self.comprovar_obstacle(14)

        elif self.estat == 14:  # Gir 90° cap al costat contrari
            if self.angle_girat() < math.pi / 2:
                cmd.twist.angular.z = -0.5 * self.direccio_esquivar
            else:
                # Gir acabat -> comprovar si hi ha obstacle nou
                self.comprovar_obstacle(15)

        elif self.estat == 15:  # Avançar per tornar a la ruta - deteccio activa
            self.cicles += 1
            if self.tipus_obstacle is not None:
                self.comprovar_obstacle(16)
            elif self.cicles < 12:
                cmd.twist.linear.x = 0.2
            else:
                # Avanç acabat -> comprovar si hi ha obstacle nou
                self.comprovar_obstacle(16)

        elif self.estat == 16:  # Gir 90° cap al costat original (redrecem)
            if self.angle_girat() < math.pi / 2:
                cmd.twist.angular.z = 0.5 * self.direccio_esquivar
            else:
                # Gir acabat -> comprovar si hi ha obstacle nou
                self.comprovar_obstacle(0)

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
