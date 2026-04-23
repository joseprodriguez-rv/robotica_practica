#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class MovimentNode(Node):
    def __init__(self):
        super().__init__('controlador_moviment')
        
        # QoS per al sensor (Segons Pràctica 4 i 5)
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # QoS per al moviment (Segons Pràctica 5)
        qos_moviment = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 1. Subscripcions i Publicador
        self.sub_laser = self.create_subscription(LaserScan, '/scan', self.laser_callback, qos_sensor)
        self.sub_comptador = self.create_subscription(Int32, '/comptador_obstacles', self.comptador_callback, 10)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', qos_moviment)
        
        self.get_logger().info('Vigilant de moviment actiu...')

        # 2. Variables de la classe (Pista: Estat del Robot)
        self.estat = 0       # 0=Avançar, 1=Girant_S, 2=Canvi_Fila, 10=Esquivar...
        self.cicles = 0      # Pista: Comptador de cicles (1 cicle = 0.1s)
        self.direccio_s = 1  # 1 = Esquerra, -1 = Dreta (per la S)
        self.objectes = 0    # Guardar quants objectes portem

    def comptador_callback(self, msg):
        # Aquest callback només escolta el node de l'odometria
        self.objectes = msg.data
        if self.objectes >= 5:
            self.estat = None # Estat final, aturem-ho tot
            self.get_logger().info('Objectiu complert: 5 objectes trobats!')

    def laser_callback(self, msg):
        # Evitem problemes si el laser no està actiu o dona errors d'inici
        if len(msg.ranges) == 0:
            return

        # 3. Creem el missatge CORRECTE (TwistStamped)
        cmd = TwistStamped()
        
        # 4. Omplim la capçalera (OBLIGATORI en Stamped)
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        # --- ESTAT FINAL (ATURAT) ---
        if self.estat == None:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            self.pub.publish(cmd)
            return

        # --- ESTAT 0: RECTE I LLEGINT LÀSER ---
        if self.estat == 0:
            # Slicing de llistes en Python (Pràctica 4.2)
            dreta = msg.ranges[330:359]
            esquerra = msg.ranges[0:30]
            con_frontal = list(dreta) + list(esquerra)
            
            # Filtrar valors invàlids (evitar 0.0 i inf)
            valors_valids = [d for d in con_frontal if d > msg.range_min and d < msg.range_max]
            
            if len(valors_valids) > 0:
                distancia_min = min(valors_valids)
            else:
                distancia_min = 10.0 # Camí lliure
                
            # Decisió segons la distància
            if distancia_min < 0.5:
                cmd.twist.linear.x = 0.0 # Frenem
                
                # Comprovem quants raigs impacten a prop per diferenciar paret d'objecte
                raigs_aprop = [d for d in valors_valids if d < 0.5]
                
                if len(raigs_aprop) > 40:
                    self.get_logger().warn('PARET! Iniciant S')
                    self.estat = 1 # Passem a fer la corba
                    self.cicles = 0 # Reiniciem el comptador de cicles
                else:
                    self.get_logger().warn('OBJECTE! Esquivant')
                    self.estat = 10 # Passem a rodejar el cub
                    self.cicles = 0
            else:
                # Si no hi ha res, avancem
                cmd.twist.linear.x = 0.2
                cmd.twist.angular.z = 0.0

        # --- MÀQUINA D'ESTATS: LA MANIOBRA EN "S" ---
        # Ús del mètode de cicles: si el laser arriba a 10Hz, cada 10 missatges és 1 segon.
        elif self.estat == 1: 
            self.cicles += 1
            if self.cicles < 30: # 3 segons girant a 0.5rad/s = aprox 90 graus
                cmd.twist.angular.z = 0.5 * self.direccio_s
            else:
                self.estat = 2
                self.cicles = 0
                
        elif self.estat == 2: # Avançar recta (canvi de fila)
            self.cicles += 1
            if self.cicles < 20: # 2 segons de canvi de fila
                cmd.twist.linear.x = 0.15
            else:
                self.estat = 3
                self.cicles = 0
                
        elif self.estat == 3: # Segon gir
            self.cicles += 1
            if self.cicles < 30:
                cmd.twist.angular.z = 0.5 * self.direccio_s
            else:
                self.direccio_s *= -1 # Invertim costat per la següent S
                self.estat = 0 # Tornem a mode exploració recta
                self.cicles = 0

        # --- MÀQUINA D'ESTATS: ESQUIVAR OBJECTE ---
        # Esquivament simple per la dreta
        elif self.estat == 10: # Gir 90º dreta
            self.cicles += 1
            if self.cicles < 30: cmd.twist.angular.z = -0.5
            else: self.estat = 11; self.cicles = 0
                
        elif self.estat == 11: # Avançar 
            self.cicles += 1
            if self.cicles < 25: cmd.twist.linear.x = 0.2
            else: self.estat = 12; self.cicles = 0
                
        elif self.estat == 12: # Gir 90º esquerra
            self.cicles += 1
            if self.cicles < 30: cmd.twist.angular.z = 0.5
            else: self.estat = 13; self.cicles = 0

        elif self.estat == 13: # Avançar (superar objecte)
            self.cicles += 1
            if self.cicles < 25: cmd.twist.linear.x = 0.2
            else: self.estat = 14; self.cicles = 0

        elif self.estat == 14: # Gir 90º esquerra
            self.cicles += 1
            if self.cicles < 30: cmd.twist.angular.z = 0.5
            else: self.estat = 15; self.cicles = 0

        elif self.estat == 15: # Avançar per tornar a la ruta
            self.cicles += 1
            if self.cicles < 25: cmd.twist.linear.x = 0.2
            else: self.estat = 16; self.cicles = 0

        elif self.estat == 16: # Gir 90º dreta (ens redrecem a la X)
            self.cicles += 1
            if self.cicles < 30: cmd.twist.angular.z = -0.5
            else: self.estat = 0; self.cicles = 0 # Tornem a començar!

        # 6. Enviem (Usant el mateix nom que a l'init)
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MovimentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ordre de parada en sortir
        stop = TwistStamped()
        stop.header.stamp = node.get_clock().now().to_msg()
        stop.twist.linear.x = 0.0 
        stop.twist.angular.z = 0.0
        node.pub.publish(stop)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
