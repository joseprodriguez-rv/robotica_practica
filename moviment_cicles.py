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

        qos_moviment = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub_laser = self.create_subscription(LaserScan, '/scan', self.laser_callback, qos_sensor)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_comptador = self.create_subscription(Int32, '/comptador_objectes', self.comptador_callback, 10)
        self.sub_tipus = self.create_subscription(String, '/tipus_obstacle', self.tipus_callback, 10)
        self.timer = self.create_timer(0.1, self.control_callback)

        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', qos_moviment)
        # NOU: Publisher d'Int32 per a la nova lògica de detecció
        self.pub_maniobra = self.create_publisher(Int32, '/en_maniobra', 10)
        self.get_logger().info('Node de moviment actiu i sincronitzat amb nova detecció...')

        self.estat = 0
        self.cicles = 0
        self.objectes = 0
        self.tipus_obstacle = None
        self.direccio_esquivar = 1  
        self.direccio_paret = 1     
        self.laser_ranges = []      

        self.angle_actual = 0.0     
        self.angle_inici_gir = None 

    def laser_callback(self, msg):
        self.laser_ranges = msg.ranges

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
        # Només processem missatges si NO estem en ple gir (estats de gir: 1, 10, 12, 14, 16)
        if self.estat not in (1, 10, 12, 14, 16):
            self.tipus_obstacle = msg.data

    def iniciar_gir(self):
        self.angle_inici_gir = self.angle_actual

    def angle_girat(self):
        if self.angle_inici_gir is None:
            return 0.0
        diff = self.angle_actual - self.angle_inici_gir
        diff = (diff + math.pi) % (2 * math.pi) - math.pi
        return abs(diff)

    def calcular_costat_lliure(self):
        # 1 = Esquerra (Z positiu), -1 = Dreta (Z negatiu)
        if len(self.laser_ranges) >= 360:
            dreta = self.laser_ranges[270:360]
            esquerra = self.laser_ranges[0:90]
            valors_validsdre = [d for d in dreta if 0.1 < d < 6]
            valors_validsesq = [d for d in esquerra if 0.1 < d < 6]
            if len(valors_validsesq) > len(valors_validsdre):
                return 1   # Més espai a l'esquerra -> Gir positiu
            else:
                return -1  # Més espai a la dreta -> Gir negatiu
        return 1

    def comprovar_obstacle_pendent(self, estat_seguent):
        if self.tipus_obstacle == 'PARET':
            self.get_logger().warn('PARET detectada -> Girant cap al costat lliure
