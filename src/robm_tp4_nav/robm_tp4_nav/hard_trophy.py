#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Range
from robm_interfaces.msg import Color, ServoCommand, GripperCommand
import math

class LineFollowerGripper(Node):
    def __init__(self):
        super().__init__('line_follower_gripper')

        # === PUBLISHERS ===
        self.pub_cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.pub_servo = self.create_publisher(ServoCommand, '/cmd_servo', 10)
        self.pub_gripper = self.create_publisher(GripperCommand, '/cmd_gripper', 10)

        # === SUBSCRIBERS ===
        self.sub_color = self.create_subscription(Color, '/color', self.color_cb, 10)
        self.sub_tof = self.create_subscription(Range, '/tof', self.tof_cb, 10)

        # === COLOR REFERENCES ===
        self.ref_white = (4.60, 17.38, 31.70)
        self.ref_gray  = (5.27, 17.19, 30.71)
        self.ref_green = (7.30, 30.82, 12.59)
        self.ref_red   = (48.03, 6.90, 8.94)
        self.ref_blue  = (1.66, 12.95, 60.33)

        # === CONFIGURATION ===
        self.v_fast = 0.10           
        self.v_slow = 0.04           
        self.v_line_drive = 0.06
        self.v_line_turn  = 0.8

        self.Kp_align = 0.42          
        self.slip_factor = 1.0       
        
        self.dist_search_limit = 0.25 
        self.dist_slow_down = 0.17    
        self.dist_grab = 0.09         
        
        self.grip_open_pos = 1.0
        self.grip_close_pos = 0.55
        self.grip_closed = False     

        # === STATE MACHINE ===
        self.state = 'ALIGN_FORWARD' 
        
        # Scan Variables
        self.scan_width = 0.9        
        self.scan_angle = -0.9
        self.scan_step = 0.05
        self.scan_complete = False   
        self.best_scan_dist = 999.0
        self.best_scan_angle = 0.0

        # ACTIVE SCANNING VARIABLES (NEW)
        self.drive_scan_angle = 0.0
        self.drive_scan_dir = 1
        self.drive_scan_limit = 0.5  # Shallow sweep (+- 0.3 rad) while driving

        self.locked_angle = 0.0      
        self.state_start_time = None
        self.rgb = (0.0, 0.0, 0.0)
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
        self.get_logger().info("🤖 Robot initialized: Active Scanning Mode")

    def color_cb(self, msg):
        self.rgb = (msg.r, msg.g, msg.b)

    def tof_cb(self, msg):
        if msg.range > 0.01:
            self.tof = msg.range

    def dist(self, a, b):
        return sum((a[i]-b[i])**2 for i in range(3))

    # --- NEW: ACTIVE SCANNING WHILE DRIVING ---
    def scan_while_driving(self):
        """Sweeps the sensor slightly left/right to catch off-center trophies."""
        self.drive_scan_angle += 0.1 * self.drive_scan_dir # Fast sweep
        
        if self.drive_scan_angle > self.drive_scan_limit:
            self.drive_scan_angle = self.drive_scan_limit
            self.drive_scan_dir = -1
        elif self.drive_scan_angle < -self.drive_scan_limit:
            self.drive_scan_angle = -self.drive_scan_limit
            self.drive_scan_dir = 1
            
        servo = ServoCommand()
        servo.angle = self.drive_scan_angle
        self.pub_servo.publish(servo)

    # --- FULL SCAN (STATIONARY) ---
    def perform_scan_sweep(self):
        limit_max = self.scan_width
        
        if self.scan_complete:
            self.scan_angle = -self.scan_width
            self.best_scan_dist = 999.0
            self.best_scan_angle = 0.0
            self.scan_complete = False

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

        # Color Classification
        dists = {
            'WHITE': min(self.dist(self.rgb, self.ref_white), self.dist(self.rgb, self.ref_gray)),
            'RED': self.dist(self.rgb, self.ref_red),
            'GREEN': self.dist(self.rgb, self.ref_green),
            'BLUE': self.dist(self.rgb, self.ref_blue)
        }
        color = min(dists, key=dists.get)
        min_dist = dists[color]

        # =========================================================
        # PHASE 1: LINE FOLLOWING (With Active Scanning)
        # =========================================================
        if self.state in ['ALIGN_FORWARD', 'ALIGN_TURN', 'FOLLOWING']:
            
            # 1. Active Scan (Look around while driving)
            self.scan_while_driving()

            # 2. Check for Trophy
            if self.tof < self.dist_search_limit and not self.grip_closed:
                self.get_logger().info("🏆 TARGET SPOTTED! Switching to Offroad Mode.")
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = 0.0
                self.state = 'TROPHY_SEARCH' 
                self.scan_complete = True    
                self.pub_cmd.publish(cmd)
                return

            # 3. Line Logic
            if self.state == 'ALIGN_FORWARD':
                if color == 'RED':
                    self.state = 'ALIGN_TURN'
                    cmd.twist.linear.x = 0.0
                    cmd.twist.angular.z = -self.v_line_turn
                else:
                    cmd.twist.linear.x = self.v_line_drive

            elif self.state == 'ALIGN_TURN':
                if color == 'GREEN':
                    self.state = 'FOLLOWING'
                    cmd.twist.linear.x = 0.0
                    cmd.twist.angular.z = 0.0
                else:
                    cmd.twist.angular.z = -self.v_line_turn

            elif self.state == 'FOLLOWING':
                if min_dist > 4000.0:
                    cmd.twist.linear.x = -0.05 
                elif color == 'BLUE':
                    cmd.twist.linear.x = 0.0
                elif color == 'WHITE':
                    cmd.twist.linear.x = self.v_line_drive
                elif color == 'RED':
                    cmd.twist.linear.x = 0.0
                    cmd.twist.angular.z = -self.v_line_turn
                elif color == 'GREEN':
                    cmd.twist.linear.x = 0.0
                    cmd.twist.angular.z = self.v_line_turn

        # =========================================================
        # PHASE 2: TROPHY HUNTING
        # =========================================================
        elif self.state == 'TROPHY_SEARCH':
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            
            scan_done = self.perform_scan_sweep()
            
            if scan_done:
                if self.best_scan_dist < self.dist_search_limit:
                    self.get_logger().info(f"Locked at {self.best_scan_angle:.2f} rad.")
                    self.locked_angle = self.best_scan_angle
                    self.state = 'TROPHY_ALIGN'
                else:
                    self.get_logger().info("Lost target. Rescanning...")
                    self.scan_complete = True 

        elif self.state == 'TROPHY_ALIGN':
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = self.Kp_align * self.locked_angle
            
            dt = 0.05
            measured_turn = (cmd.twist.angular.z * dt) * self.slip_factor
            self.locked_angle -= measured_turn

            if abs(self.locked_angle) < 0.05:
                self.get_logger().info("Aligned! CHARGING...")
                self.state = 'TROPHY_APPROACH'

        elif self.state == 'TROPHY_APPROACH':
            cmd.twist.angular.z = 0.0 
            
            # Lock Servo Forward for Approach
            servo_msg = ServoCommand()
            servo_msg.angle = 0.0
            self.pub_servo.publish(servo_msg)

            if self.tof < self.dist_grab:
                self.get_logger().info("GRABBING!")
                cmd.twist.linear.x = 0.0
                self.state = 'GRAB'
            elif self.tof < self.dist_slow_down:
                cmd.twist.linear.x = self.v_slow
            else:
                cmd.twist.linear.x = self.v_fast

        elif self.state == 'GRAB':
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            grip_msg.position = self.grip_close_pos
            self.grip_closed = True
            
            self.state = 'TURN_BACK'
            self.state_start_time = self.get_clock().now().nanoseconds / 1e9

        # =========================================================
        # PHASE 3: RETURN
        # =========================================================
        elif self.state == 'TURN_BACK':
            grip_msg.position = self.grip_close_pos
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = self.v_line_turn 
            
            elapsed = self.get_clock().now().nanoseconds / 1e9 - self.state_start_time
            if elapsed > 4.0: 
                self.get_logger().info("Returning to track...")
                self.state = 'RETURN_IGNORE_RED'

        elif self.state == 'RETURN_IGNORE_RED':
            grip_msg.position = self.grip_close_pos
            cmd.twist.linear.x = self.v_line_drive 

            # 1. SAFETY CHECK: If lost, back up! (Crucial Fix)
            if min_dist > 4000.0:
                cmd.twist.linear.x = -0.05
                cmd.twist.angular.z = 0.0
            
            # 2. GREEN DETECTED: Resume normal path
            elif color == 'GREEN':
                self.get_logger().info("🟩 Green line found! Resuming normal logic.")
                cmd.twist.linear.x = 0.0 # Stop briefly to ensure state switch handles turn correctly
                self.state = 'FOLLOWING'

            # 3. RED DETECTED: IGNORE IT (Drive Straight)
            elif color == 'RED':
                # Treat Red exactly like White
                cmd.twist.linear.x = self.v_line_drive
                cmd.twist.angular.z = 0.0
            
            # 4. WHITE DETECTED: Drive Straight
            elif color == 'WHITE':
                cmd.twist.linear.x = self.v_line_drive
                cmd.twist.angular.z = 0.0
            
            # 5. BLUE: Stop (Standard behavior)
            elif color == 'BLUE':
                cmd.twist.linear.x = 0.0

        if self.grip_closed:
            grip_msg.position = self.grip_close_pos
        else:
            grip_msg.position = self.grip_open_pos

        self.pub_gripper.publish(grip_msg)
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerGripper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()