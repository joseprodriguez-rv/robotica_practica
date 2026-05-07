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

        # --- AFEGIT: Subscripció a les parets per anul·lar falsos positius ---
        self.sub_paret = self.create_subscription(
            Odometry, '/paret_detectada', self.paret_callback, 10
        )

        self.pub_cartograf = self.create_publisher(
            Int32, '/comptador_objectes', 10
        )

    def paret_callback(self, msg):
        x_paret = msg.pose.pose.position.x
        y_paret = msg.pose.pose.position.y

        # Llindar de distància per considerar que l'objecte era en realitat una paret
        radi_anulacio = 0.5

        mapa_filtrat = []
        objectes_eliminats = 0

        # Revisem tots els objectes guardats
        for obj in self.mapa:
            dist = math.sqrt((x_paret - obj)**2 + (y_paret - obj)**2)
            if dist < radi_anulacio:
                # L'objecte està massa a prop de la paret, és un fals positiu
                objectes_eliminats += 1
                self.get_logger().warn(f'FALS POSITIU ANUL·LAT: Objecte a X={obj:.2f}, Y={obj:.2f} era paret.')
            else:
                # Si està lluny, el mantenim al mapa
                mapa_filtrat.append(obj)

        # Si hem eliminat algun objecte, actualitzem el comptador i notifiquem
        if objectes_eliminats > 0:
            self.mapa = mapa_filtrat
            self.comptador_oficial -= objectes_eliminats

            msg_comptador = Int32()
            msg_comptador.data = self.comptador_oficial
            self.pub_cartograf.publish(msg_comptador)
            self.get_logger().info(f'Nou comptador oficial: {self.comptador_oficial} objectes.')

    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # 0.35m — suficient per absorbir soroll d'odometria
        # però prou petit per distingir objectes propers (ampolles, estoigs)
        radi_proximitat = 0.35

        # filtrem deteccions repetides
        es_repetit = any(
            math.sqrt((x - obj)**2 + (y - obj)**2) < radi_proximitat
            for obj in self.mapa
        )

        if not es_repetit:
            self.mapa.append((x, y))

            self.comptador_oficial += 1
            msg_comptador = Int32()
            msg_comptador.data = self.comptador_oficial
            self.pub_cartograf.publish(msg_comptador)

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
