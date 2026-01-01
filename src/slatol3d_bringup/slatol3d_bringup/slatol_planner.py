import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String, Empty
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contacts
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class SlatolPlanner(Node):
    def __init__(self):
        super().__init__('slatol_planner')
        
        # --- INTERFACES ---
        self.create_subscription(Float64MultiArray, '/slatol/planner/goal', self.jump_command_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10) # Velocity for Drift Calc
        self.create_subscription(Contacts, '/slatol/contact', self.contact_callback, 10) # Event Detection
        self.create_subscription(String, '/slatol/mode', self.mode_callback, 10)
        self.create_subscription(Empty, '/slatol/cmd/standup', self.standup_callback, 10)
        
        self.traj_pub = self.create_publisher(JointTrajectory, '/slatol_position_controller/joint_trajectory', 10)
        
        # --- STATE ---
        self.mode = "PLANNER"
        self.state = "STANCE"
        self.pitch = 0.0
        self.pitch_rate = 0.0
        self.vx = 0.0 
        self.is_touching_ground = True
        
        self.jump_start_time = 0.0
        self.flight_start_time = 0.0
        self.thrust_timer = None
        self.target_height = 0.5
        
        # --- GAINS ---
        # ZMP (Ground)
        self.Kp_zmp = 0.8; self.Kd_zmp = 0.05
        # AMC (Air)
        self.K_amc = 1.2
        # Raibert Heuristic (Drift/Wind Rejection)
        self.K_raibert = 0.15 
        
        # Geometry
        self.L1 = 0.35; self.L2 = 0.35; self.standing_h = 0.60 
        
        self.create_timer(0.01, self.control_loop) 
        self.get_logger().info("Slatol Event-Based Controller Ready.")

    def contact_callback(self, msg):
        """Hardware Interrupt: Ground Contact"""
        # If contacts list is not empty, we hit something
        self.is_touching_ground = len(msg.contacts) > 0

    def control_loop(self):
        if self.mode != "ZMP_AMC": return
        now = self.get_clock().now().nanoseconds / 1e9
        
        # 1. Safety
        if self.state != "FALLEN" and abs(self.pitch) > 1.3:
            self.get_logger().warn("CRITICAL: Tipped Over.")
            self.state = "FALLEN"
        
        if self.state == "FALLEN": return
            
        elif self.state == "STANCE":
            # [Method] Polar ZMP (Ugurlu 2008)
            balance_offset = (self.Kp_zmp * self.pitch) + (self.Kd_zmp * self.pitch_rate)
            q_h, q_k = self.inverse_kinematics(0.0, -self.standing_h)
            if q_h: self.send_cmd(0.0, q_h + balance_offset, q_k, 0.05)
                
        elif self.state == "THRUST":
            t_elapsed = now - self.jump_start_time
            # Impulse generation
            progress = min(1.0, t_elapsed / 0.25)
            current_h = min(0.68, self.standing_h + (0.2 * progress))
            q_h, q_k = self.inverse_kinematics(0.05, -current_h) 
            if q_h: self.send_cmd(0.0, q_h, q_k, 0.01)
            
            if t_elapsed > 0.25:
                self.state = "FLIGHT"
                self.flight_start_time = now
                self.get_logger().info("LIFTOFF -> AMC Active")

        elif self.state == "FLIGHT":
            # [Method] AMC + Raibert (Ugurlu 2021 + Yim 2020)
            
            # A. AMC: Counter-rotate body
            amc_kick = self.K_amc * self.pitch_rate
            
            # B. Raibert: Place foot to stop drift (Wind/Thrust Rejection)
            # x_land = v * T_stance/2 + k * (v - v_des)
            foot_target_x = (self.vx * 0.1) + (self.K_raibert * self.vx)
            raibert_angle = foot_target_x / 0.5 
            
            q_h_nom, q_k_nom = self.inverse_kinematics(0.1, -0.50) 
            if q_h_nom:
                self.send_cmd(0.0, q_h_nom + amc_kick + raibert_angle, q_k_nom, 0.02)
            
            # C. Event-Based Landing (Robustness)
            # If flight > 0.1s AND sensor detects ground -> LAND
            if (now - self.flight_start_time) > 0.1 and self.is_touching_ground:
                self.state = "LANDING"
                self.get_logger().info(f"CONTACT DETECTED: Landing. V={self.vx:.2f}")

        elif self.state == "LANDING":
            # Dampen Impact
            q_h, q_k = self.inverse_kinematics(0.15, -0.55)
            stabilizer = (self.Kp_zmp * 2.0) * self.pitch 
            if q_h: self.send_cmd(0.0, q_h + stabilizer, q_k, 0.05)
            
            if (now - self.flight_start_time) > 1.5:
                self.state = "STANCE"

    def start_zmp_jump(self):
        if self.state != "STANCE": return
        q_h, q_k = self.inverse_kinematics(0.0, -0.45)
        if q_h: self.send_cmd(0.0, q_h, q_k, 0.5)
        self.thrust_timer = self.create_timer(0.5, self.trigger_thrust)

    def trigger_thrust(self):
        if self.thrust_timer: self.thrust_timer.cancel(); self.thrust_timer = None
        self.state = "THRUST"
        self.jump_start_time = self.get_clock().now().nanoseconds / 1e9

    def execute_nlp_trajectory(self):
        traj = JointTrajectory()
        traj.joint_names = ['hip_haa_joint', 'hip_hfe_joint', 'knee_kfe_joint']
        q_h, q_k = self.inverse_kinematics(0.0, -0.45)
        if q_h: self.add_pt(traj, [0.0, q_h, q_k], 0.5)
        q_h, q_k = self.inverse_kinematics(0.1, -0.68)
        if q_h: self.add_pt(traj, [0.0, q_h, q_k], 0.8)
        self.traj_pub.publish(traj)

    def inverse_kinematics(self, x, z):
        r = math.sqrt(x**2 + z**2)
        if r > (self.L1 + self.L2 - 0.01) or r < 0.1: return None, None
        c_k = (r**2 - self.L1**2 - self.L2**2) / (2*self.L1*self.L2)
        try:
            q_k = -math.acos(max(-1, min(1, c_k)))
            alpha = math.atan2(z, x)
            beta = math.acos((self.L1**2 + r**2 - self.L2**2)/(2*self.L1*r))
            return (alpha + beta) + (math.pi/2), q_k
        except ValueError: return None, None

    def send_cmd(self, haa, hfe, kfe, dt):
        msg = JointTrajectory()
        msg.joint_names = ['hip_haa_joint', 'hip_hfe_joint', 'knee_kfe_joint']
        self.add_pt(msg, [haa, hfe, kfe], dt)
        self.traj_pub.publish(msg)

    def add_pt(self, msg, pos, t):
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in pos]
        pt.time_from_start = Duration(sec=int(t), nanosec=int((t%1)*1e9))
        msg.points.append(pt)

    def imu_callback(self, msg):
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        sinp = 2.0 * (qw * qy - qz * qx)
        self.pitch = math.asin(max(-1.0, min(1.0, sinp)))
        self.pitch_rate = msg.angular_velocity.y

    def odom_callback(self, msg):
        self.vx = msg.twist.twist.linear.x

    def jump_command_callback(self, msg):
        if self.mode == "MANUAL": return
        self.target_height = msg.data[0]
        if self.mode == "NLP": self.execute_nlp_trajectory()
        elif self.mode == "ZMP_AMC":
            if self.state == "FALLEN": self.get_logger().warn("Robot is FALLEN.")
            else: self.start_zmp_jump()

    def standup_callback(self, msg):
        self.state = "STANCE"
        self.send_cmd(0.0, 0.0, 0.0, 1.0)
    
    def mode_callback(self, msg):
        self.mode = msg.data
        if self.mode == "ZMP_AMC" and self.state == "FALLEN" and abs(self.pitch) < 0.5: self.state = "STANCE"

def main():
    rclpy.init()
    node = SlatolPlanner()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()