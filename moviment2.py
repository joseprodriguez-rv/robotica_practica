#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32, String, Bool
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

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
        self.direccio_s = 1
        self.objectes = 0
        self.tipus_obstacle = None
        self.en_maniobra = False    # flag que s'envia a deteccio
        self.direccio_esquivar = 1  # 1=esquerra, -1=dreta (decidit pel laser)
        self.laser_ranges = []      # última lectura del làser

    # Callbacks
    def laser_callback(self, msg):
        self.laser_ranges = msg.ranges  # guardem per usar al control_callback

    def comptador_callback(self, msg):
        self.objectes = msg.data
        if self.objectes >= 5:
            self.estat = None
            self.get_logger().info('Objectiu complert: 5 objectes trobats!')

    def tipus_callback(self, msg):
        self.tipus_obstacle = msg.data  # 'PARET' o 'OBJECTE'

    def control_callback(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        # Publicar flag en_maniobra perquè deteccio sàpiga si pot publicar
        flag = Bool()
        flag.data = self.estat in (1, 2, 3)  # només durant maniobra en S
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

                # Decidir cap a quin costat esquivar segons espai lliure al làser
                if self.tipus_obstacle == 'PARET':
                    self.get_logger().warn('PARET detectada → Maniobra en S')
                    self.estat = 1
                else:
                    self.get_logger().warn('OBJECTE detectat → Esquivant')
                    self.estat = 10

                    if len(self.laser_ranges) >= 360:
                        dreta = self.laser_ranges[300:360]
                        esquerra = self.laser_ranges[0:60]
                        valors_validsdre = [d for d in dreta if 0.1 < d < 6]
                        valors_validsesq = [d for d in esquerra if 0.1 < d < 6]
                        num_dre = len(valors_validsdre)
                        num_esq = len(valors_validsesq)
                        if num_esq > num_dre: #he canviat això pq vagi bé
                            self.direccio_esquivar = 1
                        else:
                            self.direccio_esquivar = -1
                        self.get_logger().info(
                            f'Obstacle detectat → Espai lliure: ESQ={num_esq} rays, DRE={num_dre} rays → Gir {"ESQUERRA" if self.direccio_esquivar == 1 else "DRETA"}'
                        )

                self.tipus_obstacle = None
                self.cicles = 0

        #  MANIOBRA EN "S"
        elif self.estat == 1:   # Primer gir (~90°)
            self.cicles += 1
            if self.cicles < 30:
                cmd.twist.angular.z = 0.5 * self.direccio_s
            else:
                self.estat = 2
                self.cicles = 0

        elif self.estat == 2:   # Recta de canvi de fila
            self.cicles += 1
            if self.cicles < 20:
                cmd.twist.linear.x = 0.15
            else:
                self.estat = 3
                self.cicles = 0

        elif self.estat == 3:   # Segon gir (~90°, mateix sentit)
            self.cicles += 1
            if self.cicles < 30:
                cmd.twist.angular.z = 0.5 * self.direccio_s
            else:
                self.direccio_s *= -1   # Invertim per la propera S
                self.estat = 0
                self.cicles = 0
                self.tipus_obstacle = None  # reset per no processar obstacle vell

        #  ESQUIVAR OBJECTE (costat decidit pel làser)
        elif self.estat == 10:  # Gir 90° cap al costat lliure
            self.cicles += 1
            if self.cicles < 30:
                cmd.twist.angular.z = 0.5 * self.direccio_esquivar
            else:
                self.estat = 11
                self.cicles = 0

        elif self.estat == 11:  # Avançar (esquivar lateral)
            self.cicles += 1
            if self.cicles < 12:
                cmd.twist.linear.x = 0.2
            else:
                self.estat = 12
                self.cicles = 0

        elif self.estat == 12:  # Gir 90° cap al costat contrari
            self.cicles += 1
            if self.cicles < 30:
                cmd.twist.angular.z = -0.5 * self.direccio_esquivar
            else:
                self.estat = 13
                self.cicles = 0

        elif self.estat == 13:  # Avançar (superar objecte)
            self.cicles += 1
            if self.cicles < 25:
                cmd.twist.linear.x = 0.2
            else:
                self.estat = 14
                self.cicles = 0

        elif self.estat == 14:  # Gir 90° cap al costat contrari
            self.cicles += 1
            if self.cicles < 30:
                cmd.twist.angular.z = -0.5 * self.direccio_esquivar
            else:
                self.estat = 15
                self.cicles = 0

        elif self.estat == 15:  # Avançar per tornar a la ruta
            self.cicles += 1
            if self.cicles < 12:
                cmd.twist.linear.x = 0.2
            else:
                self.estat = 16
                self.cicles = 0

        elif self.estat == 16:  # Gir 90° cap al costat original (redrecem)
            self.cicles += 1
            if self.cicles < 30:
                cmd.twist.angular.z = 0.5 * self.direccio_esquivar
            else:
                self.estat = 0
                self.cicles = 0
                self.tipus_obstacle = None  # reset per no processar obstacle vell

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
