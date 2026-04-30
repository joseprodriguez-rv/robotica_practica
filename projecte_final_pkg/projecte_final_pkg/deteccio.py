#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool, Int32
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

        #subscripció a maniobra — pausar detecció durant avanços laterals
        self.en_maniobra = False
        self.sub_maniobra = self.create_subscription(
            Bool, '/en_maniobra', self.maniobra_callback, 10)

        #subscripció al comptador d'objectes
        self.objectes = 0
        self.sub_comptador = self.create_subscription(
            Int32, '/comptador_objectes', self.comptador_callback, 10)

        #publisher objecte en odometria amb x i y
        self.pub_objecte = self.create_publisher(Odometry, '/objecte_detectat', 10)
        self.pub_tipus = self.create_publisher(String, '/tipus_obstacle', 10)

        self.get_logger().info('Node de Detecció actiu')

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

    def laser_callback(self, msg):
        # pausar durant avanços laterals de l'esquiva (estats 11, 13, 15)
        if self.en_maniobra or self.objectes >= 5:
            return

        #con frontal
        part_esquerra = msg.ranges[0:61]
        part_dreta = msg.ranges[300:360]
        con_frontal = list(part_esquerra) + list(part_dreta)

        #treure valors invàlids
        distancies_valides = [d for d in con_frontal if msg.range_min < d < msg.range_max]

        if len(distancies_valides) > 0:
            distancia_min = min(distancies_valides)

            #si detectem un obstacle a prop
            if distancia_min < 0.4:
                tipus = String()
                # paret: més de 30 punts al con frontal
                # objecte petit (ampolla, estoig, cilindre): menys de 30 punts
                if len(distancies_valides) > 90:
                    tipus.data = 'PARET'
                    self.get_logger().info('PARET detectada')
                else:
                    tipus.data = 'OBJECTE'
                    num_min = msg.ranges.index(distancia_min) #per trobar l'angle on està l'objecte
                    angle = msg.angle_min + (num_min * msg.angle_increment)
                    self.get_logger().warn(f'Objecte detectat a {distancia_min:.2f}m')
                    self.enviar_posicio_objecte(distancia_min, angle)
                self.pub_tipus.publish(tipus)

    def enviar_posicio_objecte(self, r, a):
        #suma d'angle del robot + angle del laser
        angle_final = self.robot_ang + a

        obj_x = self.robot_x + (r * math.cos(angle_final))
        obj_y = self.robot_y + (r * math.sin(angle_final))

        msg_obj = Odometry()
        msg_obj.header.stamp = self.get_clock().now().to_msg()
        msg_obj.header.frame_id = 'map'
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
