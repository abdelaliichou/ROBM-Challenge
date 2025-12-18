#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Range
from robm_interfaces.msg import ServoCommand, GripperCommand
import math

class TrophyGrabber(Node):
    def __init__(self):
        super().__init__('trophy_grabber')

        # === PUBLISHERS ===
        self.pub_cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.pub_servo = self.create_publisher(ServoCommand, '/cmd_servo', 10)
        self.pub_gripper = self.create_publisher(GripperCommand, '/cmd_gripper', 10)

        # === SUBSCRIBERS ===
        self.sub_tof = self.create_subscription(Range, '/tof', self.tof_cb, 10)

        # === CONFIGURATION ===
        self.v_fast = 0.10           
        self.v_slow = 0.04           
        
        # === TUNING THE TURN ===
        self.Kp_align = 0.42          
        self.slip_factor = 1.0       
        
        # Grabbing Config
        self.dist_search_limit = 0.35 # IGNORE anything further than 35cm
        self.dist_slow_down = 0.17    # Start slowing down at 15cm
        self.dist_grab = 0.09         # STOP and GRAB at 9cm (Precision Grab)
        
        # Gripper Positions
        self.grip_open_pos = 1.0
        self.grip_close_pos = 0.55
        self.grip_closed = False     

        # === STATE MACHINE ===
        self.state = 'SEARCH'
        
        # Scan Variables
        self.scan_width = 0.8        # Wider scan (+- 0.6 rad) to find it easily
        self.scan_angle = -0.8
        self.scan_step = 0.05
        self.scan_complete = False   
        self.best_scan_dist = 999.0
        self.best_scan_angle = 0.0

        self.locked_angle = 0.0      
        self.tof = 2.0

        self.init_robot()
        self.timer = self.create_timer(0.05, self.control_loop)

    def init_robot(self):
        servo = ServoCommand()
        servo.angle = 0.0
        self.pub_servo.publish(servo)
        grip = GripperCommand()
        grip.position = self.grip_open_pos
        self.pub_gripper.publish(grip)
        self.get_logger().info("🤖 Robot initialized: Close Range Precision Mode")

    def tof_cb(self, msg):
        if msg.range > 0.01:
            self.tof = msg.range

    # --- SCANNING LOGIC ---
    def perform_scan_sweep(self):
        limit_max = self.scan_width
        
        if self.scan_complete:
            self.scan_angle = -self.scan_width
            self.best_scan_dist = 999.0
            self.best_scan_angle = 0.0
            self.scan_complete = False

        # Only record if it is a VALID close object (< 25cm)
        if self.tof < self.dist_search_limit:
            if self.tof < self.best_scan_dist:
                self.best_scan_dist = self.tof
                self.best_scan_angle = self.scan_angle

        self.scan_angle += self.scan_step
        servo = ServoCommand()
        servo.angle = self.scan_angle
        self.pub_servo.publish(servo)

        if self.scan_angle >= limit_max:
            servo = ServoCommand()
            servo.angle = 0.0
            self.pub_servo.publish(servo)
            self.scan_complete = True 
            return True 

        return False 

    def control_loop(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        grip_msg = GripperCommand()

        # 1. SEARCH
        if self.state == 'SEARCH':
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            
            scan_done = self.perform_scan_sweep()
            
            if scan_done:
                # Did we find something close (< 25cm)?
                if self.best_scan_dist < self.dist_search_limit:
                    self.get_logger().info(f"Target locked at {self.best_scan_angle:.2f} rad. Aligning...")
                    self.locked_angle = self.best_scan_angle
                    self.state = 'ALIGN'
                else:
                    self.get_logger().info("No close target found. Rescanning...")
                    self.scan_complete = True 

        # 2. ALIGN
        elif self.state == 'ALIGN':
            cmd.twist.linear.x = 0.0
            
            # Turn Command
            cmd.twist.angular.z = self.Kp_align * self.locked_angle
            
            # Virtual Odometry
            dt = 0.05
            measured_turn = (cmd.twist.angular.z * dt) * self.slip_factor
            self.locked_angle -= measured_turn

            if abs(self.locked_angle) < 0.05:
                self.get_logger().info("Aligned! CHARGING forward...")
                self.state = 'APPROACH'

        # 3. APPROACH
        elif self.state == 'APPROACH':
            cmd.twist.angular.z = 0.0 
            
            # Keep servo centered
            servo_msg = ServoCommand()
            servo_msg.angle = 0.0
            self.pub_servo.publish(servo_msg)

            # === DISTANCE CONTROL ===
            if self.tof < self.dist_grab:
                # Perfect Grab Distance (9cm)
                self.get_logger().info(f"Target reached ({self.tof:.2f}m). GRABBING!")
                cmd.twist.linear.x = 0.0
                self.state = 'GRAB'

            elif self.tof < self.dist_slow_down:
                # Creep forward very slowly (0.04)
                cmd.twist.linear.x = self.v_slow
            
            else:
                # Approach fast
                cmd.twist.linear.x = self.v_fast

        # 4. GRAB
        elif self.state == 'GRAB':
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            grip_msg.position = self.grip_close_pos
            self.grip_closed = True
            self.state = 'DONE'

        # 5. DONE
        elif self.state == 'DONE':
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            grip_msg.position = self.grip_close_pos

        if self.grip_closed:
            grip_msg.position = self.grip_close_pos
        else:
            grip_msg.position = self.grip_open_pos

        self.pub_gripper.publish(grip_msg)
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TrophyGrabber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()