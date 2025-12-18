#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""robm_tp4_nav/control_heading.py - Commande proportionnelle du cap

Ce noeud permet de contrôler l'orientation du robot.
Il met en oeuvre une commande proportionnelle en vitesse de rotation.
"""

import rclpy
from rclpy.node import Node

from math import pi, atan2

from geometry_msgs.msg import TwistStamped, PoseStamped, Quaternion
from nav_msgs.msg import Odometry

def yaw_from_quaternion_msg(q: Quaternion) -> float:
    """Extract the yaw angle from a geometry_msgs/Quaternion message"""
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return atan2(siny_cosp, cosy_cosp)


def sawtooth(angle: float) -> float:
    """Wrap an angle value within the interval [-pi; +pi["""
    return (angle + pi) % (2*pi) - pi


class ControlHeadingNode(Node):
    def __init__(self):
        """Constructeur de la classe ControlHeadingNode"""
        super().__init__('control_heading')

        # cap désiré
        self.theta_d = None

        # Gain proportionnel (À ajuster si le robot oscille ou est trop lent)
        self.Kp = 1 

        # subscribers et publishers
        self._sub_goal = self.create_subscription(PoseStamped, "goal_pose", self.goal_callback, 1)
        self._sub_odom = self.create_subscription(Odometry, "odometry", self.odom_callback, 1)
        
        # NOTE: Utilisation de TwistStamped ici, crucial pour le bridge
        self._cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 1)
    

    def goal_callback(self, goal: PoseStamped):
        """Callback du mise à jour du cap désiré depuis le but défini dans Rviz"""
        # Extract goal orientation from msg
        self.theta_d = yaw_from_quaternion_msg(goal.pose.orientation)
        self.get_logger().info(f"Goal set to yaw={self.theta_d:.2f} rad ({self.theta_d*180/pi:.1f}°)")


    def odom_callback(self, odom: Odometry):
        """Callback de reception de message d'odometrie.
        Le calcul de la commande proportionnelle est effectué dans 
        cette fonction.
        """
        
        # Ne fait rien si le cap désiré n'est pas encore défini
        if self.theta_d is None:
            return
        
        # Récupère l'angle de cap courant du robot depuis l'odométrie
        theta = yaw_from_quaternion_msg(odom.pose.pose.orientation)

        # 1. Calculer l'erreur de cap (différence entre cap désiré et cap actuel)
        raw_error = self.theta_d - theta
        
        # Utilisation de sawtooth pour gérer le modulo 2*pi
        err_theta = sawtooth(raw_error)
        
        # 2. Calculer la vitesse de rotation à appliquer
        # Commande proportionnelle : u = Kp * erreur
        w = 0.0
        
        # Seuil de tolérance (zone morte) : 0.05 rad (environ 3 degrés)
        # Si on est assez proche, on s'arrête pour éviter d'osciller
        if abs(err_theta) > 0.05:
            w = self.Kp * err_theta
            
            # Saturation optionnelle (vitesse max 2.0 rad/s)
            max_rot_speed = 2.0
            if w > max_rot_speed: w = max_rot_speed
            if w < -max_rot_speed: w = -max_rot_speed
        else:
            w = 0.0

        # Envoi de la commande sur le topic ROS
        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.header.frame_id = "base_link"
        vel_msg.twist.linear.x = 0.0
        vel_msg.twist.linear.y = 0.0
        vel_msg.twist.angular.z = w
        self._cmd_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControlHeadingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()