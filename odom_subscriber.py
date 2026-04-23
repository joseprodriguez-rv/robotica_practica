
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
#Aqui canvies el import 
from nav_msgs.msg import Odometry 
#Treure el durability policy 
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class OdometrySubscriber(Node):
    def __init__(self):
        super().__init__('lector_odometry')
        
        # Canvi de LaserScan i /scan
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        
        self.get_logger().info('Llegint posició per odometria...')

    def odom_callback(self, msg):
        # editem el callback amb el nou tipo de format que odereix odom
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        vel_x = msg.twist.twist.linear.x

        #Al igual que amb lidar mostrem missatge, en aquest cas on som:        
        self.get_logger().info(f"Posició -> X: {pos_x:.2f}m | Y: {pos_y:.2f}m | Velocitat: {vel_x:.2f} m/s")

def main(args=None):
    rclpy.init(args=args)
    node = OdometrySubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()