#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int32
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class DeteccioNode(Node):
    def __init__(self):
        super().__init__('deteccio')

        #guardar on està el robot
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_ang = 0.0

        # qos per al sensor
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        #subs al laser
        self.sub_laser = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile)

        #subs a l'odom
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        #subscripció a maniobra 0=explorant, 1=girant (no detecta), 2=esquiva activat (llindar reduït)
        self.en_maniobra = 0
        self.sub_maniobra = self.create_subscription(
            Int32, '/en_maniobra', self.maniobra_callback, 10)

        #subscripció al comptador d'objectes
        self.objectes = 0
        self.sub_comptador = self.create_subscription(
            Int32, '/comptador_objectes', self.comptador_callback, 10)

        #publisher objecte en odometria amb x i y (coords)
        self.pub_objecte = self.create_publisher(Odometry, '/objecte_detectat', 10)
        self.pub_tipus = self.create_publisher(String, '/tipus_obstacle', 10)
		
		#per avisar per la terminal 
        self.get_logger().info('Node de detecció actiu...')

    def odom_callback(self, msg):
        #posició actual
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        #conversió a Yaw
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        self.robot_ang = math.atan2(siny_cosp, cosy_cosp)

    def maniobra_callback(self, msg):
        self.en_maniobra = msg.data

    def comptador_callback(self, msg):
        self.objectes = msg.data

    def classificar_obstacle(self, msg, distancia_min):
        #marges per distingir entre paret i objecte 
        llindar_centre = 0.40
        llindar_anell = 0.60

        #+-20º (40 rajos)
        #com havíem fet en una tasca anterior
        centre = list(msg.ranges[0:20]) + list(msg.ranges[340:360])
        centre_valids = [d for d in centre if msg.range_min < d < msg.range_max]
        propers_centre = [d for d in centre_valids if d < llindar_centre]

        #anell (de 20º a 60º, 80 rajos)
        anell = list(msg.ranges[20:60]) + list(msg.ranges[300:340])
        anell_valids = [d for d in anell if msg.range_min < d < msg.range_max]
        propers_anell = [d for d in anell_valids if d < llindar_anell]

        #les proporcions 
        prop_centre = len(propers_centre) / len(centre_valids) if centre_valids else 0.0
        prop_anell = len(propers_anell) / len(anell_valids) if anell_valids else 0.0


        centre_ple = prop_centre > 0.40
        anell_ple = prop_anell > 0.40
		
		#per la terminal
        self.get_logger().info(
            f'[CLASS] centre={len(propers_centre)}/{len(centre_valids)} ({prop_centre:.0%})  '
            f'anell={len(propers_anell)}/{len(anell_valids)} ({prop_anell:.0%})'
        )
		#obstacle ocupa el con frontal
        if centre_ple and anell_ple:
            return 'PARET'
        #anell bastant lliure, però tenim alguna cosa al centre
        elif (prop_centre > 0.25 and not anell_ple) or (anell_ple and not prop_centre > 0.25):
            return 'OBJECTE'
		#no ens decantem per cap opció
        else:
            return ''

    def laser_callback(self, msg):
        #treure la detecció durant els girs o quan superem els 5 objectes
        if self.en_maniobra == 1 or self.objectes >= 5:
            return

        #con frontal
        if self.en_maniobra == 2:
            part_esquerra = msg.ranges[0:20]
            part_dreta = msg.ranges[340:360]
        else:
            part_esquerra = msg.ranges[0:60]
            part_dreta = msg.ranges[300:360]

        con_frontal = list(part_esquerra) + list(part_dreta)

        #treure valors invàlids
        distancies_valides = [d for d in con_frontal if msg.range_min < d < msg.range_max]

        if len(distancies_valides) > 0:
            distancia_min = min(distancies_valides)
            llindar = 0.25 if self.en_maniobra == 2 else 0.30

            #si detectem un obstacle a prop
            if distancia_min < llindar:
                tipus = String()
                tipus.data = self.classificar_obstacle(msg, distancia_min)

                if tipus.data == 'PARET':
                    self.get_logger().info('PARET detectada')
                    self.pub_tipus.publish(tipus)
                elif tipus.data == 'OBJECTE':
                    num_min = msg.ranges.index(distancia_min)
                    angle = msg.angle_min + (num_min * msg.angle_increment)
                    self.get_logger().warn(f'Objecte detectat a {distancia_min:.2f}m')
                    self.enviar_posicio_objecte(distancia_min, angle)
                    self.pub_tipus.publish(tipus)
                else:
                    pass

    def enviar_posicio_objecte(self, r, a):
        #suma d'angle del robot + angle del laser
        angle_final = self.robot_ang + a

        obj_x = self.robot_x + (r * math.cos(angle_final))
        obj_y = self.robot_y + (r * math.sin(angle_final))

        msg_obj = Odometry()
        msg_obj.header.stamp = self.get_clock().now().to_msg()
        msg_obj.pose.pose.position.x = float(obj_x)
        msg_obj.pose.pose.position.y = float(obj_y)

        self.pub_objecte.publish(msg_obj)

def main(args=None):
    rclpy.init(args=args)
    node = DeteccioNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Aturant node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#es llegeix la posició amb msg.pose.pose.position.x
