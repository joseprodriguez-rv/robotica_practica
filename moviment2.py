#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32, String, Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math 
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

#paràmetres per poder-los ajustar
vel = 0.15
w = 0.4
dist_esq = 0.28
dist_superar = 0.45
ang_esq = math.pi/2 #90
ang_paret = math.pi*3/4 #135
marge_angle = 0.05

estat_gir = (1,10,12,14,16)
estat_avanç=(11,13,15)

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
        self.sub_odom      = self.create_subscription(Odometry,  '/odom',               self.odom_callback,      10)

        # Publicador
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', qos_moviment)
        self.pub_maniobra = self.create_publisher(Bool, '/en_maniobra', 10)  # flag per a deteccio
        self.get_logger().info('Node de moviment actiu...')

        #  Variables per saber estat
        self.estat = 0
        #trec cicles
        self.direccio_paret = 1
        self.objectes = 0
        self.tipus_obstacle = None
        self.en_maniobra = False    # flag que s'envia a deteccio
        self.direccio_esquivar = 1  # 1=esquerra, -1=dreta (decidit pel laser)
        self.laser_ranges = []      # última lectura del làser
        self.angle_actual = 0.0 #afegeixo
        self.angle_inici_gir = None #afegeixo

        #afegeixo odometria per mesurar distàncies
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_x_inici = 0.0
        self.pos_y_inici = 0.0
        

    # Callbacks
    def laser_callback(self, msg):
        self.laser_ranges = list(msg.ranges)  # guardem per usar al control_callback
   
    def odom_callback(self, msg): #afegeixo odom
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
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
        if self.estat in estats_gir:
            return 
        if self.estat is None:
            return
        #durant avançaments i recta fer deteccions
        self.tipus_obstacle = msg.data  # 'PARET' o 'OBJECTE'
    #per l'odometria
    def iniciar_gir(self):
        self.angle_inici_gir = self.angle_actual
 
    def angle_girat(self):
        if self.angle_inici_gir is None:
            return 0.0
        diff = self.angle_actual - self.angle_inici_gir
        diff = (diff + math.pi) % (2 * math.pi) - math.pi
        return abs(diff)
 
    def iniciar_avanc(self):
        self.pos_x_inici = self.pos_x
        self.pos_y_inici = self.pos_y
 
    def distancia_avancada(self):
        dx = self.pos_x - self.pos_x_inici
        dy = self.pos_y - self.pos_y_inici
        return math.sqrt(dx * dx + dy * dy)
        
    def calcular_costat_lliure(self): #per trobar +1 (esq) o -1 (dreta)
        if len(self.laser_ranges) < 360:
            return 1
 
        esquerra = self.laser_ranges[60:120]
        dreta = self.laser_ranges[240:300]
 
        num_esq = len([d for d in esquerra if 0.1 < d < 6])
        num_dre = len([d for d in dreta     if 0.1 < d < 6])
 
        self.get_logger().info(f'Punts esq: {num_esq} | Punts dre: {num_dre}')
 
        if num_esq >= num_dre:
            self.get_logger().info('-> ESQUERRA (+1)')
            return 1
        else:
            self.get_logger().info('-> DRETA (-1)')
            return -1
   
    def comprovar_obstacle_pendent(self, estat_seguent):
        if self.tipus_obstacle == 'PARET':
            self.get_logger().warn('PARET -> Girant 135°')
            self.direccio_paret = self.calcular_costat_lliure()
            self.tipus_obstacle = None
            self.estat = 1
            self.iniciar_gir()
 
        elif self.tipus_obstacle == 'OBJECTE':
            self.get_logger().warn('OBJECTE -> Esquivant')
            self.direccio_esquivar = self.calcular_costat_lliure()
            self.tipus_obstacle = None
            self.estat = 10
            self.iniciar_gir()
 
        else:
            self.tipus_obstacle = None
            self.estat = estat_seguent
            if estat_seguent in estat_gir:
                self.iniciar_gir()
            elif estat_seguent in estat_avanç:
                self.iniciar_avanc()

    def control_callback(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        # Publicar flag en_maniobra perquè deteccio sàpiga si pot publicar
        flag = Bool()
        flag.data = self.estat in estats_gir  # només durant maniobra en S
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
                cmd.twist.linear.x = vel
                cmd.twist.angular.z = 0.0
            else:
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = 0.0
                self.comprovar_obstacle_pendent(0)
        #estat 1 (gir 135 per paret)
        elif self.estat == 1:
            if self.angle_girat()<ang_paret-marge_angle:
                cmd.twist.angular.z = w * self.direccio_paret
            else:
                self.get_logger().info('Gir paret acabat')
                self.comprovar_obstacle_pendent(0)

        elif self.estat == 10:
            if self.angle_girat() < ang_esq - marge_angle:
                cmd.twist.angular.z = w * self.direccio_esquivar
            else:
                self.get_logger().info('Estat 10 acabat')
                self.comprovar_obstacle_pendent(11)
 
        elif self.estat == 11:
            # Obstacle detectat durant l'avanç lateral -> aturar i gestionar
            if self.tipus_obstacle is not None:
                cmd.twist.linear.x = 0.0
                self.get_logger().warn('Obstacle durant estat 11 -> gestionant')
                self.comprovar_obstacle_pendent(11)
            elif self.distancia_avancada() < dist_esq:
                cmd.twist.linear.x =vel
            else:
                self.get_logger().info('Estat 11 acabat')
                self.comprovar_obstacle_pendent(12)
 
        elif self.estat == 12:
            if self.angle_girat() < ang_esq - marge_angle:
                cmd.twist.angular.z = -w * self.direccio_esquivar
            else:
                self.get_logger().info('Estat 12 acabat')
                self.comprovar_obstacle_pendent(13)
 
        elif self.estat == 13:
            # Obstacle detectat durant l'avanç frontal -> aturar i gestionar
            if self.tipus_obstacle is not None:
                cmd.twist.linear.x = 0.0
                self.get_logger().warn('Obstacle durant estat 13 -> gestionant')
                self.comprovar_obstacle_pendent(13)
            elif self.distancia_avancada() < dist_superar:
                cmd.twist.linear.x = vel
            else:
                self.get_logger().info('Estat 13 acabat')
                self.comprovar_obstacle_pendent(14)
 
        elif self.estat == 14:
            if self.angle_girat() < ang_esq - marge_angle:
                cmd.twist.angular.z = -w * self.direccio_esquivar
            else:
                self.get_logger().info('Estat 14 acabat')
                self.comprovar_obstacle_pendent(15)
 
        elif self.estat == 15:
            # Obstacle detectat tornant a la ruta -> aturar i gestionar
            if self.tipus_obstacle is not None:
                cmd.twist.linear.x = 0.0
                self.get_logger().warn('Obstacle durant estat 15 -> gestionant')
                self.comprovar_obstacle_pendent(15)
            elif self.distancia_avancada() < dist_esq:
                cmd.twist.linear.x = vel
            else:
                self.get_logger().info('Estat 15 acabat')
                self.comprovar_obstacle_pendent(16)
 
        elif self.estat == 16:
            if self.angle_girat() < ang_esq - marge_angle:
                cmd.twist.angular.z = w * self.direccio_esquivar
            else:
                self.get_logger().info('Estat 16 acabat -> tornant a estat 0')
                self.tipus_obstacle = None
                self.comprovar_obstacle_pendent(0)
 
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
