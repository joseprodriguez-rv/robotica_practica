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

        #subscripció a detecció per saber les posicions dels objectes i poder comptar-los
        self.sub_deteccio = self.create_subscription(
            Odometry, '/objecte_detectat', self.callback, 10
        )
        #creació del publisher comptador
        self.pub_cartograf = self.create_publisher(
            Int32, '/comptador_objectes', 10
        )

    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        #ens serveix per distingir objectes propers (ampolles, estoigs) i absorvir el soroll
        radi_proximitat = 0.30

        # filtrem deteccions repetides
        es_repetit = any(
            math.sqrt((x - obj[0])**2 + (y - obj[1])**2) < radi_proximitat
            for obj in self.mapa
        )
        #si no és repetit el podem posar en el mapa i actualitzar i publicar el comptador
        if not es_repetit:
            self.mapa.append((x, y))

            self.comptador_oficial += 1
            msg_comptador = Int32()
            msg_comptador.data = self.comptador_oficial
            self.pub_cartograf.publish(msg_comptador)

            #per la terminal
            self.get_logger().info(f'Objecte #{self.comptador_oficial} registrat a X={x:.2f}, Y={y:.2f}')
        else:
            self.get_logger().info(f'Nova detecció massa prop. Descartat a X={x:.2f}, Y={y:.2f}')

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
