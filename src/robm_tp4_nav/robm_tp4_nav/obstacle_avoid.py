#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Range
import math

class ObstacleAvoidNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoid')

        self.sub_cmd = self.create_subscription(TwistStamped, 'cmd_vel_desired', self.cmd_callback, 10)
        self.sub_tof = self.create_subscription(Range, 'tof', self.tof_callback, 10)
        self.pub_cmd = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        self.current_range = 2.0
        
        # --- 1. PARAMÈTRES DE SÉCURITÉ ---
        self.dist_slow = 0.60   # On ralentit tôt
        self.dist_stop = 0.30   # On s'arrête loin du mur
        self.wait_time = 3.0    # Délai avant manoeuvre
        
        # --- 2. PARAMÈTRES DE MANOEUVRE ---
        self.escape_distance = 0.40 
        self.escape_speed = 0.20    # Vitesse réduite (demande utilisateur)
        self.scan_speed = 1.0       # Vitesse de rotation moyenne
        
        self.escape_duration = self.escape_distance / self.escape_speed

        # MACHINE A ÉTATS : NORMAL -> SCANNING -> ESCAPING -> REALIGNING -> NORMAL
        self.state = 'NORMAL'
        
        # Timers et Mémoire
        self.block_start_time = None
        self.state_start_time = None
        
        # Pour mémoriser combien on a tourné, afin de faire le chemin inverse
        self.scan_duration_memory = 0.0

    def tof_callback(self, msg: Range):
        # Gestion zone aveugle : si 0.0, on garde la vieille valeur
        if msg.range > 0.01:
            self.current_range = msg.range

    def cmd_callback(self, msg: TwistStamped):
        current_time = self.get_clock().now()
        dt_seconds = current_time.nanoseconds / 1e9
        
        safe_cmd = TwistStamped()
        safe_cmd.header = msg.header

        # =========================================================
        # ETAT 1 : SCANNING (Tourner jusqu'à trouver une issue)
        # =========================================================
        if self.state == 'SCANNING':
            # On stocke le temps de départ au début de l'état
            if self.state_start_time is None:
                self.state_start_time = dt_seconds

            # Condition de fin : La voie est libre avec une MARGE (+10cm)
            # Cela évite de s'arrêter pile quand le capteur ne voit plus le mur
            if self.current_range > (self.dist_slow + 0.10):
                # On calcule combien de temps on a tourné
                self.scan_duration_memory = dt_seconds - self.state_start_time
                
                self.get_logger().info(f"Voie libre après {self.scan_duration_memory:.2f}s de rotation. Avance...")
                
                # Transition vers ESCAPING
                self.state = 'ESCAPING'
                self.state_start_time = dt_seconds # Reset timer pour l'étape suivante
                return
            else:
                # Action : Tourner à Gauche
                safe_cmd.twist.linear.x = 0.0
                safe_cmd.twist.angular.z = self.scan_speed
                self.pub_cmd.publish(safe_cmd)
                return

        # =========================================================
        # ETAT 2 : ESCAPING (Avancer dans la nouvelle direction)
        # =========================================================
        elif self.state == 'ESCAPING':
            # Sécurité : Si on fonce dans un autre mur
            if self.current_range < self.dist_stop:
                self.get_logger().warn("Mur détecté pendant l'avance ! On re-scanne.")
                self.state = 'SCANNING'
                self.state_start_time = None
                return

            elapsed = dt_seconds - self.state_start_time
            
            if elapsed < self.escape_duration:
                # Action : Avancer tout droit
                safe_cmd.twist.linear.x = self.escape_speed
                safe_cmd.twist.angular.z = 0.0
                self.pub_cmd.publish(safe_cmd)
                return
            else:
                # Fin de l'avance -> On passe au RÉALIGNEMENT
                self.get_logger().info("Avance terminée. Réalignement (Retour direction initiale)...")
                self.state = 'REALIGNING'
                self.state_start_time = dt_seconds
                return

        # =========================================================
        # ETAT 3 : REALIGNING (Le point crucial du TP)
        # "Réorienter le robot dans sa direction d’avant contournement"
        # =========================================================
        elif self.state == 'REALIGNING':
            elapsed = dt_seconds - self.state_start_time
            
            # On tourne à DROITE (signe négatif) pendant la même durée qu'au SCAN
            if elapsed < self.scan_duration_memory:
                safe_cmd.twist.linear.x = 0.0
                safe_cmd.twist.angular.z = -self.scan_speed # Sens inverse
                self.pub_cmd.publish(safe_cmd)
                return
            else:
                self.get_logger().info("Réalignement terminé. Retour au pilote normal.")
                self.state = 'NORMAL'
                self.block_start_time = None
                # On laisse le code NORMAL reprendre la main ci-dessous

        # =========================================================
        # ETAT 4 : NORMAL (Surveillance)
        # =========================================================
        
        d = self.current_range
        cmd_linear = msg.twist.linear.x
        
        # 1. Freinage
        if d < self.dist_stop:
            safe_cmd.twist.linear.x = 0.0 # Arrêt net
            safe_cmd.twist.angular.z = 0.0 # Pas de rotation si trop près
        elif d < self.dist_slow:
            # Ralentissement progressif
            ratio = (d - self.dist_stop) / (self.dist_slow - self.dist_stop)
            safe_cmd.twist.linear.x = cmd_linear * ratio
            safe_cmd.twist.angular.z = msg.twist.angular.z
        else:
            safe_cmd.twist.linear.x = cmd_linear
            safe_cmd.twist.angular.z = msg.twist.angular.z

        # 2. Détection Blocage 3 secondes
        if cmd_linear > 0.0 and safe_cmd.twist.linear.x == 0.0:
            if self.block_start_time is None:
                self.block_start_time = dt_seconds
            
            elif (dt_seconds - self.block_start_time) > self.wait_time:
                self.get_logger().warn("Bloqué > 3s. Lancement Manoeuvre !")
                self.state = 'SCANNING'
                self.state_start_time = None # Important pour initier le scan
        else:
            self.block_start_time = None

        self.pub_cmd.publish(safe_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()