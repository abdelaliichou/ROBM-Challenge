#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from math import pi, atan2, sqrt, cos

from geometry_msgs.msg import TwistStamped, PoseStamped, Quaternion
from nav_msgs.msg import Odometry

def yaw_from_quaternion_msg(q: Quaternion) -> float:
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return atan2(siny_cosp, cosy_cosp)

def sawtooth(angle: float) -> float:
    return (angle + pi) % (2*pi) - pi

class MoveNode(Node):
    def __init__(self):
        super().__init__('move')
        
        self.goal_x = None
        self.goal_y = None
        self.goal_theta = None # Cap final désiré (optionnel selon le sujet, mais utile)

        # Paramètres de gains
        self.Kp_ang = 1.5  # Gain angulaire (rotation) [cite: 226]
        self.Kp_lin = 0.5  # Gain linéaire (vitesse d'avance)
        self.v_max = 0.3   # Vitesse max (m/s) pour ne pas aller trop vite [cite: 260]

        # Abonnements / Publications
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(Odometry, '/odometry', self.odom_callback, 10)
        self._cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        self.get_logger().info("Noeud Move prêt. Attente d'un Goal Pose...")

    def goal_callback(self, msg: PoseStamped):
        # On récupère la position X, Y cible
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        # On récupère aussi l'orientation finale désirée (la flèche dans RViz)
        self.goal_theta = yaw_from_quaternion_msg(msg.pose.orientation)
        
        self.get_logger().info(f"Nouveau but reçu : x={self.goal_x:.2f}, y={self.goal_y:.2f}")

    def odom_callback(self, msg: Odometry):
        if self.goal_x is None:
            return

        # 1. Position actuelle du robot
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_theta = yaw_from_quaternion_msg(msg.pose.pose.orientation)

        # 2. Calcul de la distance au but [cite: 247, 248]
        dx = self.goal_x - current_x
        dy = self.goal_y - current_y
        distance = sqrt(dx**2 + dy**2)

        cmd = TwistStamped()
        cmd.header = msg.header
        cmd.header.frame_id = "base_link"

        # 3. Condition d'arrêt : Si on est à moins de 5 cm (0.05m) 
        if distance < 0.05:
            # On est arrivé !
            # Optionnel : s'aligner sur l'orientation finale demandée par la flèche RViz [cite: 243]
            err_final = sawtooth(self.goal_theta - current_theta)
            if abs(err_final) > 0.05:
                cmd.twist.angular.z = self.Kp_ang * err_final
                cmd.twist.linear.x = 0.0
            else:
                cmd.twist.angular.z = 0.0
                cmd.twist.linear.x = 0.0
                # self.get_logger().info("Arrivé au but !")
            
            self._cmd_pub.publish(cmd)
            return

        # 4. Calcul du cap à suivre pour atteindre le point (angle du vecteur mouvement) 
        target_heading = atan2(dy, dx)
        
        # Erreur de cap (vers le point)
        err_theta = sawtooth(target_heading - current_theta)

        # 5. Calcul de la vitesse angulaire (Rotation)
        cmd.twist.angular.z = self.Kp_ang * err_theta

        # 6. Calcul de la vitesse linéaire (Avance) [cite: 258]
        # v = K * distance
        v_linear = self.Kp_lin * distance

        # Sécurité : Ralentir si on ne regarde pas le but [cite: 260]
        # Si l'erreur d'angle est grande, cos(err) diminue, donc la vitesse diminue.
        # Si on est dos au but (err > pi/2), cos est négatif -> le robot reculerait ou s'arrêterait.
        # Ici on prend max(0, cos) pour ne pas reculer, juste s'arrêter et tourner.
        v_linear = v_linear * max(0.0, cos(err_theta))

        # Saturation : Ne pas dépasser v_max [cite: 260]
        if v_linear > self.v_max:
            v_linear = self.v_max

        cmd.twist.linear.x = v_linear

        self._cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MoveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()