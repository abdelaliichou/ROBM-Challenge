#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""robm_tp4_nav/control_heading.py - Commande proportionnele du cap

Ce noeud permet de contrôler l'orientation du robot.
Il met en oeuvre une commande proportionnelle en vitesse de rotation.

S'abonne à :
    - goal_pose (geometry_msgs/PoseStamped): but (seul le cap 
        est pris en compte)
    - odometry (nav_msgs/Odometry): pose du robot calculée par odométrie
Publie :
    - cmd_vel (geometry_msgs/TwistStamped): commande en vitesse
"""

import rclpy
from rclpy.node import Node

from math import pi, cos, sin, atan2, sqrt

from geometry_msgs.msg import TwistStamped, PoseStamped, Quaternion
from nav_msgs.msg import Odometry

def yaw_from_quaternion_msg( q: Quaternion ) -> float:
    """Extract the yaw angle from a geometry_msgs/Quaternion message"""
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return atan2(siny_cosp, cosy_cosp)


def sawtooth( angle: float ) -> float:
    """Wrap an angle value within the interval [-pi; +pi["""
    return (angle + pi) % (2*pi) - pi


class ControlHeadingNode(Node):
    def __init__(self):
        """Constructeur de la classe ControlHeadingNode"""
        super().__init__('control_heading')

        # cap désiré
        self.theta_d = None

        # subscribers et publishers
        self._sub_goal = self.create_subscription(PoseStamped, "goal_pose", self.goal_callback, 1)
        self._sub_odom = self.create_subscription(Odometry, "odometry", self.odom_callback, 1)
        self._cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 1)
    

    def goal_callback(self, goal: PoseStamped):
        """Callback du mise à jour du cap désiré depuis le but défini dans Rviz"""
        
        # Extract goal orientation from msg
        self.theta_d = yaw_from_quaternion_msg(goal.pose.orientation)
        self.get_logger().info(f"Goal set to yaw={self.theta_d:.2f} rad ({self.theta_d*180/pi:.1f}°)")


    def odom_callback(self, odom: Odometry):
        """Callback de reception de message d'odometrie.
        Le calcul de la commande proprtionnelle est effectué dans 
        cette fonction.
        """
        
        # Ne fait rien si le cap désiré n'est pas encore défini
        if self.theta_d == None:
            return
        
        # Récupère l'angle de cap courant du robot depuis l'odométrie
        theta = yaw_from_quaternion_msg(odom.pose.pose.orientation)

        # TODO: Calculer l'erreur de cap (différence entre cap désiré et cap actuel)
        err_theta = 0.0  # TODO 
        
        # TODO: Calculer la vitesse de rotation à appliquer
        # - Commande proportionnelle à l'erreur
        # - Arrêt si suffisamment proche du but
        w = 0.0  # TODO
        
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
