    #definir un init amb subscriber al lidar, subscriber a l'odometria i publisher de velocitat
    #callback lidar amb msg.ranges que detecti els obstacles i decideixi on girar
    #callback odom quan es detecta un objecte i el flag està lliure s'agafa la x y del robot
       #+ la de l'obstacle
    #timer amb create_timer que miri si hi ha obstacle, sino publicar velocitat i sino angular

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import TwistStamped 
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry 
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy 

class Moviment(Node):
    def __init__(self):
        super().__init__('??') #CANVIAR
        qos_profile = QoSProfile( 
        reliability=ReliabilityPolicy.RELIABLE, 
        durability=DurabilityPolicy.VOLATILE, 
        history=HistoryPolicy.KEEP_LAST, 
        depth=10 
        ) 
        # Publicador de TwistStamped 
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10) 
        self.timer = self.create_timer(0.1, self.timer_callback) 
        self.get_logger().info('Publicant TwistStamped a /cmd_vel (Format ROS 2 Jazzy)') 

        #variables per l'estat
        self.dist_frontal=10
        self.pos_x=0.0
        self.pos_y=0.0
        self.num_obj=0
        self.obj_detectat= False #flag que diu l'enunciat
    
    def laser_callback(self,msg):
        dreta = msg.ranges[330:359]
        esquerra = msg.ranges[0:30]
        con_frontal = list(dreta) + list(esquerra)
        valors_valids = [d for d in con_frontal if d > msg.range_min and d < msg.range_max]
        self.dist_frontal = min(valors_valids) #per saber dist del què tenim + a prop
        #aqui hem d'aplicar la logica de decisió i tot el que haguem de fer

    def odom_callback(self, msg):
        # editem el callback amb el nou tipo de format que odereix odom
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        vel_x = msg.twist.twist.linear.x


def main(args=None):
    rclpy.init(args=args)
    node = Moviment()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': 
    main()
