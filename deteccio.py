#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class DeteccioNode(Node):
    def __init__(self):
        super().__init__('deteccio')
        
        #guardar on està el robot p5
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0 
        
        # qos per al sensor (Pràctica 4.2 )
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            history=HistoryPolicy.KEEP_LAST, 
            depth=10
        )
        
        #p4.2 subs al laser
        self.sub_laser = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_sensor)
        
        #subs a l'odom (P5)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        #publisher objecte en Pose2D amb x i y
        self.pub_objecte = self.create_publisher(Odometry, '/objecte_detectat', 10)

        self.get_logger().info('Node de Detecció actiu')
#he tret qos del moviment perquè aquí no moc el robot  

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
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def laser_callback(self, msg):
        #con frontal
        part_esquerra = msg.ranges[0:61]  
        part_dreta = msg.ranges[300:360]     
        con_frontal = list(part_esquerra) + list(part_dreta)
     
        #treure valors invàlids
        distancies_valides = [d for d in con_frontal if msg.range_min < d < msg.range_max]
        
        if len(distancies_valides) > 0:
            distancia_min = min(distancies_valides)
            
            #si detectem un obstacle a prop 
            if distancia_min < 0.5:
                #buscar l'angle
                num_min = msg.ranges.index(distancia_min)
                angle = msg.angle_min + (num_min * msg.angle_increment)
                
                #si és mur o objecte
                if len(distancies_valides) > 90: 
                    self.get_logger().info('PARET detectada')
                else: 
                    self.get_logger().warn(f'Objecte detectat a {distancia_min:.2f}m')
                    self.enviar_posicio_objecte(distancia_min, angle)
        else: 
            distancia_min = 10.0

    def enviar_posicio_objecte(self, r, a):
        #suma d'angle del robot + angle del laser
        angle_final = self.robot_yaw + a
        
        obj_x = self.robot_x + (r * math.cos(angle_final))
        obj_y = self.robot_y + (r * math.sin(angle_final))

        msg_obj = Odometry()
        msg_obj.header.stamp = self.get_clock().now().to_msg()
        msg_obj.header.frame_id = 'map'
        msg_obj.x = float(obj_x)
        msg_obj.y = float(obj_y)
        
        
        self.pub_objecte.publish(msg_obj)
        self.get_logger().info(f'Objecte detectat a: X={obj_x:.2f}, Y={obj_y:.2f}')

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
