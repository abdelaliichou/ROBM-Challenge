#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Range
from robm_interfaces.msg import Color  # Import du message personnalisé
from robm_interfaces.msg import ServoCommand, GripperCommand # Pour initialiser le robot
import math

class LineFollowerPusher(Node):
    def __init__(self):
        super().__init__('line_follower_pusher')

        # --- 1. PUBLISHERS & SUBSCRIBERS ---
        self.pub_cmd = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.pub_servo = self.create_publisher(ServoCommand, 'cmd_servo', 10)
        self.pub_gripper = self.create_publisher(GripperCommand, 'cmd_gripper', 10)

        self.sub_color = self.create_subscription(Color, 'color', self.color_callback, 10)
        self.sub_tof = self.create_subscription(Range, 'tof', self.tof_callback, 10)

        # --- 2. INITIALISATION ROBOT (Tourelle & Pince) ---
        # On s'assure que le capteur ToF regarde devant (0.0) [cite: 24]
        # On ferme un peu la pince pour ne pas qu'elle traîne
        self.create_timer(1.0, self.init_robot_state)
        self.robot_initialized = False

        # --- 3. PARAMÈTRES SUIVI DE LIGNE (P-CONTROLLER) ---
        self.Kp = 3.5           # Gain Proportionnel (À ajuster : plus grand = virage plus sec)
        self.base_speed = 0.15  # Vitesse de croisière
        self.color_r = 0.0
        self.color_g = 0.0
        self.color_b = 0.0

        # --- 4. PARAMÈTRES OBSTACLE (BULLDOZER) ---
        self.tof_range = 2.0
        self.push_trigger_dist = 0.10  # Déclenche la poussée à 10cm
        self.push_speed = 0.35         # Vitesse rapide pour pousser lourd
        self.push_duration = 3.0       # Durée de la poussée (secondes)
        self.backup_duration = 1.5     # Durée du recul après poussée

        # --- 5. MACHINE À ÉTATS ---
        # États : 'FOLLOWING', 'PUSHING', 'RECOVERING'
        self.state = 'FOLLOWING'
        self.state_start_time = None

        # Boucle de contrôle principale (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

    def init_robot_state(self):
        """Envoie une commande unique pour centrer la tourelle."""
        if not self.robot_initialized:
            # Centrer la tourelle [cite: 24]
            msg_servo = ServoCommand()
            msg_servo.angle = 0.0 
            self.pub_servo.publish(msg_servo)
            
            # Mettre la pince en position neutre/fermée [cite: 19]
            msg_grip = GripperCommand()
            msg_grip.position = 0.2
            self.pub_gripper.publish(msg_grip)
            
            self.get_logger().info("Robot Initialisé : Tourelle centrée.")
            self.robot_initialized = True

    def color_callback(self, msg: Color):
        self.color_r = msg.r
        self.color_g = msg.g
        self.color_b = msg.b

    def tof_callback(self, msg: Range):
        # Gestion zone aveugle
        if msg.range > 0.01:
            self.tof_range = msg.range

    def control_loop(self):
        current_time = self.get_clock().now()
        dt = current_time.nanoseconds / 1e9
        
        cmd = TwistStamped()
        cmd.header.stamp = current_time.to_msg()
        cmd.header.frame_id = 'base_link'

        # =========================================================
        # ÉTAT 1 : FOLLOW_LINE (Suivi de ligne)
        # =========================================================
        if self.state == 'FOLLOWING':
            # A. Vérification Obstacle
            if self.tof_range < self.push_trigger_dist:
                self.get_logger().warn("OBSTACLE DÉTECTÉ ! MODE BULLDOZER ACTIVÉ 🚜")
                self.state = 'PUSHING'
                self.state_start_time = dt
                # Arrêt net avant de charger
                cmd.twist.linear.x = 0.0
                self.pub_cmd.publish(cmd)
                return

            # B. Calcul de l'erreur (Rouge vs Vert)
            # Rouge à Gauche, Vert à Droite 
            # Si G > R : On est trop à Droite -> Erreur Positive -> Tourner à Gauche (z > 0)
            # Si R > G : On est trop à Gauche -> Erreur Négative -> Tourner à Droite (z < 0)
            error = self.color_g - self.color_r
            
            # Commande P-Controller
            angular_z = self.Kp * error
            
            # On avance, mais on ralentit si le virage est serré
            linear_x = self.base_speed
            if abs(error) > 0.3: # Si on est très mal aligné
                linear_x = 0.05  # On ralentit pour mieux tourner

            cmd.twist.linear.x = linear_x
            cmd.twist.angular.z = angular_z

        # =========================================================
        # ÉTAT 2 : PUSHING (Foncer dans le tas)
        # =========================================================
        elif self.state == 'PUSHING':
            elapsed = dt - self.state_start_time
            
            if elapsed < self.push_duration:
                # On fonce tout droit, fort ! 
                cmd.twist.linear.x = self.push_speed
                cmd.twist.angular.z = 0.0
            else:
                self.get_logger().info("Poussée terminée. Recul tactique...")
                self.state = 'RECOVERING'
                self.state_start_time = dt
                return

        # =========================================================
        # ÉTAT 3 : RECOVERING (Reculer après l'effort)
        # =========================================================
        elif self.state == 'RECOVERING':
            elapsed = dt - self.state_start_time
            
            if elapsed < self.backup_duration:
                # Marche arrière lente
                cmd.twist.linear.x = -0.15
                cmd.twist.angular.z = 0.0
            else:
                self.get_logger().info("Zone dégagée ? Retour au suivi de ligne.")
                self.state = 'FOLLOWING'
                self.tof_range = 2.0 # Reset virtuel pour éviter re-déclenchement immédiat
                return

        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerPusher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()