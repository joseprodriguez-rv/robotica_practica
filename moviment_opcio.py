#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32
import math

class MovimentNode(Node):
    def __init__(self):
        super().__init__('controlador_moviment')
        
        # --- MÀQUINA D'ESTATS ---
        self.estat = 'AVANÇAR'
        self.comptador_obstacles = 0
        
        # Variables de memòria per les maniobres basades en temps
        self.temps_inici_estat = self.get_clock().now()
        self.direccio_s = 1 # 1 per començar la "S" girant a l'esquerra, -1 per la dreta
        
        # --- COMUNICACIÓ ---
        self.sub_laser = self.create_subscription(LaserScan, '/scan', self.laser_cb, 10)
        
        # Ens subscrivim al tòpic on l'altre fitxer publicarà quants objectes ha validat
        self.sub_comptador = self.create_subscription(Int32, '/comptador_obstacles', self.comptador_cb, 10)
        
        self.pub_cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        self.get_logger().info('🚗 Node de moviment actiu. Mode exploració en S i esquiva iniciat.')

    def comptador_cb(self, msg):
        """Aquest callback rep el recompte des de l'altre fitxer (el cartògraf)"""
        self.comptador_obstacles = msg.data
        if self.comptador_obstacles >= 5 and self.estat != 'ATURAT':
            self.get_logger().info('🛑 Hem rebut el senyal de 5 obstacles! Aturant el robot.')
            self.canviar_estat('ATURAT')

    def canviar_estat(self, nou_estat):
        """Funció d'ajuda per canviar d'estat i reiniciar el cronòmetre"""
        self.estat = nou_estat
        self.temps_inici_estat = self.get_clock().now()
        self.get_logger().info(f'🔄 Canvi d\'estat: {nou_estat}')

    def obtenir_temps_estat(self):
        """Retorna quants segons portem en l'estat actual"""
        return (self.get_clock().now() - self.temps_inici_estat).nanoseconds / 1e9

    def crear_cmd(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        return cmd

    def laser_cb(self, msg):
        cmd = self.crear_cmd()
        
        if self.estat == 'ATURAT':
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            self.pub_cmd.publish(cmd)
            return

        # 1. MODO AVANÇAR: Busquem obstacles
        if self.estat == 'AVANÇAR':
            # Agafem un con frontal ampli (de -45 a 45 graus) per veure si és un objecte o una paret llarga
            front_esquerra = msg.ranges[0:45]
            front_dreta = msg.ranges[315:359]
            con_frontal = list(front_esquerra) + list(front_dreta)
            
            # Filtrem els raigs que estan a prop (menys de 0.5 metres)
            raigs_a_prop = [d for d in con_frontal if msg.range_min < d < 0.5]
            
            if len(raigs_a_prop) > 0: # Hi ha alguna cosa a menys de 0.5m
                # LÒGICA PER DIFERENCIAR PARET DE CUB
                # Si hi ha molts raigs bloquejats (> 40), vol dir que l'obstacle és molt ample (Paret)
                # Si n'hi ha pocs, és un objecte petit (Cub)
                if len(raigs_a_prop) > 40:
                    self.get_logger().warn('🧱 Paret detectada! Iniciant corba en S.')
                    self.canviar_estat('S_GIR_1')
                else:
                    self.get_logger().warn('📦 Objecte detectat! Iniciant maniobra d\'esquivament.')
                    self.canviar_estat('ESQ_GIR_1')
            else:
                cmd.twist.linear.x = 0.2
                cmd.twist.angular.z = 0.0

        # ==========================================
        # 2. MANIOBRA EN "S" (Canvi de Fila)
        # ==========================================
        # Gir 90 graus -> Avançar una mica -> Gir 90 graus
        elif self.estat == 'S_GIR_1':
            if self.obtenir_temps_estat() < 3.14: # 3.14 segons a 0.5 rad/s = ~90 graus
                cmd.twist.angular.z = 0.5 * self.direccio_s
            else:
                self.canviar_estat('S_AVANÇAR')
                
        elif self.estat == 'S_AVANÇAR':
            if self.obtenir_temps_estat() < 2.0: # Avançar 2 segons (per fer la fila més ampla)
                cmd.twist.linear.x = 0.15
            else:
                self.canviar_estat('S_GIR_2')
                
        elif self.estat == 'S_GIR_2':
            if self.obtenir_temps_estat() < 3.14: 
                cmd.twist.angular.z = 0.5 * self.direccio_s
            else:
                # Hem acabat la S. Invertim la direcció per la propera paret i tornem a avançar
                self.direccio_s *= -1 
                self.canviar_estat('AVANÇAR')

        # ==========================================
        # 3. MANIOBRA ESQUIVAR OBJECTE (Rodejar a la dreta)
        # ==========================================
        # Exemple hardcoded per esquivar: Gira Dreta -> Avança -> Gira Esq -> Avança -> Gira Esq -> Avança -> Gira Dreta
        elif self.estat == 'ESQ_GIR_1':
            if self.obtenir_temps_estat() < 3.14: cmd.twist.angular.z = -0.5 # 90º Dreta
            else: self.canviar_estat('ESQ_AVANÇAR_1')
                
        elif self.estat == 'ESQ_AVANÇAR_1':
            if self.obtenir_temps_estat() < 1.5: cmd.twist.linear.x = 0.2
            else: self.canviar_estat('ESQ_GIR_2')
                
        elif self.estat == 'ESQ_GIR_2':
            if self.obtenir_temps_estat() < 3.14: cmd.twist.angular.z = 0.5 # 90º Esquerra
            else: self.canviar_estat('ESQ_AVANÇAR_2')
                
        elif self.estat == 'ESQ_AVANÇAR_2':
            if self.obtenir_temps_estat() < 2.5: cmd.twist.linear.x = 0.2 # Passem de llarg l'objecte
            else: self.canviar_estat('ESQ_GIR_3')
                
        elif self.estat == 'ESQ_GIR_3':
            if self.obtenir_temps_estat() < 3.14: cmd.twist.angular.z = 0.5 # 90º Esquerra
            else: self.canviar_estat('ESQ_AVANÇAR_3')
                
        elif self.estat == 'ESQ_AVANÇAR_3':
            if self.obtenir_temps_estat() < 1.5: cmd.twist.linear.x = 0.2 # Tornem a la línia original
            else: self.canviar_estat('ESQ_GIR_4')
                
        elif self.estat == 'ESQ_GIR_4':
            if self.obtenir_temps_estat() < 3.14: cmd.twist.angular.z = -0.5 # 90º Dreta (Ens posem rectes de nou)
            else: self.canviar_estat('AVANÇAR')

        # Publicar moviment
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MovimentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stop = node.crear_cmd()
        stop.twist.linear.x = 0.0
        stop.twist.angular.z = 0.0
        node.pub_cmd.publish(stop)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()