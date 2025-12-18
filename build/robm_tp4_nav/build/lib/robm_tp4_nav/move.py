#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""robm_tp4_nav/move.py - Navigation vers un point

Ce noeud permet de faire naviguer le robot jusqu'à un but spécifié dans
RViz.

S'abonne à :
    - goal_pose (geometry_msgs/PoseStamped): but
    - odometry (nav_msgs/Odometry): pose du robot calculée par odométrie
Publie :
    - cmd_vel (geometry_msgs/TwistStamped): commande en vitesse
"""

import rclpy
from rclpy.node import Node
from math import pi, cos, sin, atan2, sqrt

from geometry_msgs.msg import TwistStamped, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

def yaw_from_quaternion_msg( q: Quaternion ) -> float:
    """Extract the yaw angle from a geometry_msgs/Quaternion message"""
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return atan2(siny_cosp, cosy_cosp)

def sawtooth( angle: float ) -> float:
    """Wrap an angle value within the interval [-pi; +pi["""
    return (angle + pi) % (2*pi) - pi


class MoveNode(Node):
    def __init__(self):
        super().__init__('move')

        # Position désirée
        self.x_d: float|None = None
        self.y_d: float|None = None
        # Cap désiré
        self.theta_d: float|None = None
        # Publishers et subscribers
        self._sub_goal = self.create_subscription(PoseStamped, "goal_pose", self.goal_callback, 1)
        self._sub_odom = self.create_subscription(Odometry, "odometry", self.odom_callback, 10)
        self._cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 1)


    def goal_callback(self, goal: PoseStamped):
        """Callback du mise à jour du but"""
        self.get_logger().info(f"Goal set to x={goal.pose.position.x:.02f} y={goal.pose.position.y:.02f}")
        # Extract goal position and orientation from msg
        self.x_goal      = goal.pose.position.x
        self.y_goal      = goal.pose.position.y
        self.theta_goal = yaw_from_quaternion_msg(goal.pose.orientation)


    def odom_callback(self, odom):
        """Callback de reception de message d'odometrie.
        Le calcul de la commande est effectué dans cette fonction.
        """
        # Ne rien faire si le but n'est pas connu
        if self.x_goal == None or self.y_goal == None or self.theta_goal == None:
            return
        
        # Extract current position and orientation from msg
        x     = odom.pose.pose.position.x
        y     = odom.pose.pose.position.y
        theta = yaw_from_quaternion_msg(odom.pose.pose.orientation)

        # TODO: Calculer la distance au but
        dx = 0.0 # TODO
        dy = 0.0 # TODO
        d = 0.0 # TODO
        
        # TODO: Calculer le cap à suivre pour aller au but
        theta_d = 0.0 # TODO
        
        # TODO: Calculer la vitesse angulaire du robot (cf. control_heading.py)
        w = 0.0 # TODO
        
        # TODO: Calculer la vitesse linéaire du robot
        vx = 0.0 # TODO

        # La vitesse laterale reste nulle
        vy = 0.0
        
        # Publish speed command (in not zero for too long)
        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.header.frame_id = "base_link"
        vel_msg.twist.linear.x = vx
        vel_msg.twist.linear.y = vy
        vel_msg.twist.angular.z = w
        self._cmd_pub.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MoveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()