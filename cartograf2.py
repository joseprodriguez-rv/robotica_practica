import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
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
        #subscripció a maniobra
        self.en_maniobra = False
        self.sub_maniobra = self.create_subscription(
            Bool, '/en_maniobra', self.maniobra_callback, 10)

        self.pub_cartograf = self.create_publisher(
            Int32, '/comptador_objectes', 10
        )

    def callback(self, msg):
        def maniobra_callback(self, msg):
        self.en_maniobra = msg.data
      
        # 1. Posició (això ja ho tenieu)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        radi_proximitat = 0.2

        # filtrem deteccions repetides per veure que no guardem la mateixa,
        # si hi ha soroll, també les hem de filtrar
        es_repetit = any(
            math.sqrt((x - obj[0])**2 + (y - obj[1])**2) < radi_proximitat # distància < llindar?
            for obj in self.mapa
        )

        if not es_repetit and not self.en_maniobra: #que només afegeixi a la llista si no està en maniobra
            self.mapa.append((x, y))

            self.comptador_oficial += 1
            msg_comptador = Int32()
            msg_comptador.data = self.comptador_oficial
            self.pub_cartograf.publish(msg_comptador)

            self.get_logger().info(f'Objecte #{self.comptador_oficial} registrat a X={x:.2f}, Y={y:.2f}')

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
