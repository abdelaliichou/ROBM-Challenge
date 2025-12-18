import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Range
from robm_interfaces.msg import Color, ServoCommand, GripperCommand
import math
import time

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

        # === REFERENCES ===
        self.ref_white = (4.60, 17.38, 31.70)
        self.ref_gray  = (5.27, 17.19, 30.71)
        self.ref_green = (7.30, 30.82, 12.59)
        self.ref_red   = (48.03, 6.90, 8.94)
        self.ref_blue  = (1.66, 12.95, 60.33)

        # === SPEEDS ===
        self.v_forward = 0.06
        self.v_rotate  = 0.8
        self.v_trophy  = 0.06  # Vitesse d'avance par étapes

        # === LOST MODE ===
        self.lost_threshold = 4000.0

        # === GRIPPER PARAMETERS ===
        self.grip_open_pos = 1.0
        self.grip_close_pos = 0.55
        self.grip_counter = 0
        self.grip_delay_ticks = 5
        self.grip_closed = False

        # === STATE MACHINE ===
        self.state = 'ALIGN_FORWARD'
        self.state_start_time = None
        
        # === VARIABLES ETAPE PAR ETAPE ===
        self.step_target_angle = 0.0
        self.step_timer = 0.0
        
        # Scan vars
        self.scan_angle = 0.0
        self.scan_dir = 1
        self.scan_best_dist = 999.0
        self.scan_best_angle = 0.0
        self.scan_complete = False

        # === SENSOR DATA ===
        self.rgb = (0.0, 0.0, 0.0)
        self.tof = 2.0

        # === INIT ROBOT ===
        self.init_robot()

        # === MAIN LOOP ===
        self.timer = self.create_timer(0.05, self.control_loop)

    # ------------------------------------------------
    def init_robot(self):
        servo = ServoCommand()
        servo.angle = 0.0
        self.pub_servo.publish(servo)

        grip = GripperCommand()
        grip.position = self.grip_open_pos
        self.pub_gripper.publish(grip)

        self.get_logger().info("🤖 Robot initialized (servo=0, gripper=OPEN)")

    # ------------------------------------------------
    def color_cb(self, msg):
        self.rgb = (msg.r, msg.g, msg.b)

    def tof_cb(self, msg):
        if msg.range > 0.01:
            self.tof = msg.range

    # ------------------------------------------------
    def dist(self, a, b):
        return sum((a[i]-b[i])**2 for i in range(3))

    # ------------------------------------------------
    def perform_scan_sweep(self):
        """ Balaye le servo de gauche à droite pour trouver le trophée """
        # Reset au début du scan
        if self.scan_complete:
            self.scan_best_dist = 999.0
            self.scan_best_angle = 0.0
            self.scan_angle = -0.5
            self.scan_complete = False
            
        # Avance le servo
        self.scan_angle += 0.05
        
        # Mémorise le meilleur point
        if self.tof < self.scan_best_dist:
            self.scan_best_dist = self.tof
            self.scan_best_angle = self.scan_angle
            
        # Commande Servo
        servo = ServoCommand()
        servo.angle = self.scan_angle
        self.pub_servo.publish(servo)
        
        # Fin du balayage
        if self.scan_angle >= 0.5:
            self.scan_complete = True
            # Recentrer le servo pour visu
            servo.angle = 0.0
            self.pub_servo.publish(servo)
            return True # Scan fini
            
        return False # Scan en cours

    # ------------------------------------------------
    def control_loop(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        grip_msg = GripperCommand()

        # --- Color classification ---
        dists = {
            'WHITE': min(self.dist(self.rgb, self.ref_white), self.dist(self.rgb, self.ref_gray)),
            'RED': self.dist(self.rgb, self.ref_red),
            'GREEN': self.dist(self.rgb, self.ref_green),
            'BLUE': self.dist(self.rgb, self.ref_blue)
        }
        color = min(dists, key=dists.get)
        min_dist = dists[color]

        # ================= STATE MACHINE =================
        
        # --- PHASE 1: SUIVI DE LIGNE ---
        if self.state in ['ALIGN_FORWARD', 'ALIGN_TURN', 'FOLLOWING']:
            
            # Detection Trophée (Distance 50cm)
            if self.tof < 0.50 and not self.grip_closed:
                self.get_logger().info("🏆 Trophy DETECTED -> Stopping to Scan")
                # On arrête tout immédiatement
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = 0.0
                self.pub_cmd.publish(cmd)
                
                # Passage en mode ETAPE 1 : SCAN
                self.state = 'STEP_SCAN'
                self.scan_complete = True # Pour forcer le reset au premier appel
                return

            # Logique Suivi de Ligne
            if self.state == 'ALIGN_FORWARD':
                if color == 'RED':
                    self.get_logger().info("🔴 RED -> Align Turn")
                    self.state = 'ALIGN_TURN'
                    cmd.twist.linear.x = 0.0
                    cmd.twist.angular.z = -self.v_rotate
                else:
                    cmd.twist.linear.x = self.v_forward

            elif self.state == 'ALIGN_TURN':
                if color == 'GREEN':
                    self.get_logger().info("🟩 GREEN -> Following")
                    self.state = 'FOLLOWING'
                    cmd.twist.linear.x = 0.0
                    cmd.twist.angular.z = 0.0
                else:
                    cmd.twist.angular.z = -self.v_rotate

            elif self.state == 'FOLLOWING':
                if min_dist > self.lost_threshold:
                    cmd.twist.linear.x = -0.05 # Recul sécurité
                elif color == 'BLUE':
                    cmd.twist.linear.x = 0.0
                elif color == 'WHITE':
                    cmd.twist.linear.x = self.v_forward
                elif color == 'RED':
                    cmd.twist.linear.x = 0.0
                    cmd.twist.angular.z = -self.v_rotate
                elif color == 'GREEN':
                    cmd.twist.linear.x = 0.0
                    cmd.twist.angular.z = self.v_rotate

        # --- PHASE 2: APPROCHE ETAPE PAR ETAPE ---
        
        # ETAPE A: SCAN (ARRET COMPLET)
        elif self.state == 'STEP_SCAN':
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            
            # Exécute le scan
            scan_done = self.perform_scan_sweep()
            
            if scan_done:
                self.step_target_angle = self.scan_best_angle
                # Si on a trouvé quelque chose de cohérent (<60cm)
                if self.scan_best_dist < 0.60:
                    # self.get_logger().info(f"Target found at {self.step_target_angle:.2f} rad")
                    self.state = 'STEP_ALIGN'
                else:
                    # Rien trouvé ? On recommence le scan
                    self.scan_complete = True

        # ETAPE B: ALIGN (ROTATION SUR PLACE)
        elif self.state == 'STEP_ALIGN':
            cmd.twist.linear.x = 0.0
            
            error = self.step_target_angle
            
            # Si erreur significative (> 3 deg), on tourne
            if abs(error) > 0.05:
                cmd.twist.angular.z = 1.0 * error # Proportionnel
            else:
                # Aligné ! On passe au mouvement
                # self.get_logger().info("Aligned -> Moving Forward")
                cmd.twist.angular.z = 0.0
                self.state = 'STEP_MOVE'
                self.state_start_time = self.get_clock().now().nanoseconds / 1e9

        # ETAPE C: MOVE (AVANCE TOUT DROIT)
        elif self.state == 'STEP_MOVE':
            # On avance tout droit pendant 0.5 seconde (un "pas")
            cmd.twist.linear.x = self.v_trophy
            cmd.twist.angular.z = 0.0 # Surtout pas de rotation ici !
            
            # Vérification continue de la saisie
            # Si on est très près (10cm), on attrape
            if self.tof < 0.10:
                self.state = 'GRAB'
                return

            # Timer du pas (0.5s)
            elapsed = self.get_clock().now().nanoseconds / 1e9 - self.state_start_time
            if elapsed > 0.5:
                # Fin du pas, on s'arrête et on re-scanne pour corriger la trajectoire
                cmd.twist.linear.x = 0.0
                self.state = 'STEP_SCAN'
                self.scan_complete = True # Reset scan

        # ETAPE D: GRAB
        elif self.state == 'GRAB':
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            
            self.grip_counter += 1
            if self.grip_counter >= self.grip_delay_ticks:
                self.get_logger().info("🦀 GOTCHA! -> Closing")
                grip_msg.position = self.grip_close_pos
                self.grip_closed = True
                self.state = 'TURN_BACK'
                self.state_start_time = self.get_clock().now().nanoseconds / 1e9
            
        # --- PHASE 3: RETOUR ---
        elif self.state == 'TURN_BACK':
            grip_msg.position = self.grip_close_pos # Garde fermé
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = self.v_rotate
            
            elapsed = self.get_clock().now().nanoseconds / 1e9 - self.state_start_time
            if elapsed > 4.0:
                self.state = 'RETURN_TO_TRACK'

        elif self.state == 'RETURN_TO_TRACK':
            grip_msg.position = self.grip_close_pos
            cmd.twist.linear.x = self.v_forward
            if color == 'RED' or color == 'GREEN':
                self.get_logger().info("🔄 Back on track")
                self.state = 'FOLLOWING'

        # Gestion Pince par défaut (ouverte sauf si closed)
        if not self.grip_closed:
            grip_msg.position = self.grip_open_pos
        else:
            grip_msg.position = self.grip_close_pos

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