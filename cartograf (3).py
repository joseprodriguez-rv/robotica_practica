#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
import math

class CartografNode(Node):
    def __init__(self):
        super().__init__('cartograf')

        self.mapa = []
        self.comptador_oficial = 0

        self.sub_deteccio = self.create_subscription(
            Odometry, '/objecte_detectat', self.callback, 10
        )

        self.pub_cartograf = self.create_publisher(
            Int32, '/comptador_objectes', 10
        )

    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Ignorem missatges sense timestamp vàlid (frame_id buit = missatge no inicialitzat)
        if msg.header.frame_id == '':
            return

        # Radi de 0.35 m: absorbeix soroll d'odometria però distingeix objectes propers
        radi_proximitat = 0.35

        es_repetit = any(
            math.sqrt((x - obj[0])**2 + (y - obj[1])**2) < radi_proximitat
            for obj in self.mapa
        )

        if not es_repetit:
            self.mapa.append((x, y))
            self.comptador_oficial += 1

            msg_comptador = Int32()
            msg_comptador.data = self.comptador_oficial
            self.pub_cartograf.publish(msg_comptador)

            self.get_logger().info(
                f'Objecte #{self.comptador_oficial} registrat a X={x:.2f}, Y={y:.2f}'
            )

            # Mostrem el mapa complet cada cop que s'afegeix un objecte nou
            self.get_logger().info(f'Mapa actual: {self.mapa}')

def main(args=None):
    rclpy.init(args=args)
    node = CartografNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
