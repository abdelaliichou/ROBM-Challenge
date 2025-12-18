import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Range
from robm_interfaces.msg import Color, GripperCommand
import time

class LineFollowerGripper(Node):
    def __init__(self):
        super().__init__('line_follower_gripper')

        # === PUBLISHERS ===
        self.pub_cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.pub_gripper = self.create_publisher(GripperCommand, '/cmd_gripper', 10)

        # === SUBSCRIBERS ===
        self.sub_color = self.create_subscription(Color, '/color', self.color_cb, 10)
        self.sub_tof = self.create_subscription(Range, '/tof', self.tof_cb, 10)

        # === REFERENCES ===
        self.ref_white = (4.60, 17.38, 31.70)
        self.ref_gray  = (5.27, 17.19, 30.71)
        self.ref_green = (7.30, 30.82, 12.59)
        self.ref_red   = (48.03, 6.90, 8.94)
        self.ref_blue  = (1.66, 12.95, 60.33)

        # === SPEEDS ===
        self.lost_threshold = 4000.0
        self.v_forward = 0.06
        self.v_rotate  = 0.8

        # === SENSOR DATA ===
        self.rgb = (0.0, 0.0, 0.0)
        self.tof = 2.0

        # Gripper state
        self.grip_closed = False
        self.grip_open_pos = 1.0  # more open for trophy
        self.grip_close_pos = 0.50
        self.grip_delay_ticks = 4  # delay before closing (10Hz timer)
        self.grip_counter = 0

        # Timer
        self.timer = self.create_timer(0.05, self.control_loop)

    # ----------------------------
    def color_cb(self, msg):
        self.rgb = (msg.r, msg.g, msg.b)

    def tof_cb(self, msg):
        if msg.range > 0.01:
            self.tof = msg.range

    def dist(self, a, b):
        return sum((a[i] - b[i])**2 for i in range(3))

    # ----------------------------
    def control_loop(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        # ========= LINE FOLLOWING =========
        dists = {
            'WHITE': min(self.dist(self.rgb, self.ref_white),
                         self.dist(self.rgb, self.ref_gray)),
            'RED':   self.dist(self.rgb, self.ref_red),
            'GREEN': self.dist(self.rgb, self.ref_green),
            'BLUE':  self.dist(self.rgb, self.ref_blue)
        }
        color = min(dists, key=dists.get)
        min_dist = dists[color]

        # ========== LOST MODE ==========
        if min_dist > self.lost_threshold:
            self.get_logger().warn("⚠️ LOST MODE → Reversing slightly")
            cmd.twist.linear.x = -0.05
            cmd.twist.angular.z = 0.0

        # ========== LINE FOLLOWING ==========
        elif color == 'BLUE':
            self.get_logger().warn("🟦 BLUE DETECTED → EMERGENCY STOP")
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0

        elif color == 'WHITE':
            self.get_logger().info("⬜ WHITE → FORWARD")
            cmd.twist.linear.x = self.v_forward
            cmd.twist.angular.z = 0.0

        elif color == 'RED':
            self.get_logger().info("🟥 RED → TURN RIGHT")
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = -self.v_rotate  # turn right

        elif color == 'GREEN':
            self.get_logger().info("🟩 GREEN → TURN LEFT")
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = self.v_rotate   # turn left

        # ========= TROPHY GRIPPER =========
        grip_msg = GripperCommand()

        if self.tof < 0.09 and not self.grip_closed:
            # Trophy detected → start delay counter
            self.grip_counter += 1
            if self.grip_counter >= self.grip_delay_ticks:
                self.get_logger().info("🏆 TROPHY DETECTED → CLOSING GRIPPER")
                grip_msg.position = self.grip_close_pos
                self.grip_closed = True
            else:
                # Keep open during delay
                grip_msg.position = self.grip_open_pos
        elif self.tof >= 0.15 and self.grip_closed:
            # Trophy removed → open gripper
            self.get_logger().info("No trophy → OPENING GRIPPER")
            grip_msg.position = self.grip_open_pos
            self.grip_closed = False
            self.grip_counter = 0
        else:
            # Maintain previous state
            grip_msg.position = self.grip_open_pos if not self.grip_closed else self.grip_close_pos

        self.pub_cmd.publish(cmd)
        self.pub_gripper.publish(grip_msg)

# ----------------------------
def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerGripper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
