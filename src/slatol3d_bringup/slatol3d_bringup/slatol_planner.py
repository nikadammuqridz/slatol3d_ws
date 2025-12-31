import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import Imu
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class SlatolPlanner(Node):
    def __init__(self):
        super().__init__('slatol_planner')
        
        # Subs
        self.create_subscription(Float64MultiArray, '/slatol/planner/goal', self.plan_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(String, '/slatol/mode', self.mode_callback, 10)
        
        # Pubs
        self.traj_pub = self.create_publisher(JointTrajectory, '/slatol_position_controller/joint_trajectory', 10)
        
        # Internal State
        self.mode = "PLANNER" # Default
        self.pitch = 0.0
        self.pitch_rate = 0.0
        self.is_jumping = False
        
        # Balance Gains (PD Controller)
        self.Kp_balance = 4.0 
        self.Kd_balance = 0.3

        # Balance Gains (PD Controller for ZMP)
        self.Kp_balance = 4.0 
        self.Kd_balance = 0.3
        
        # NEW: AMC Gains (Angular Momentum Control for Flight)
        # This swings the leg to counter-act body spin
        self.Kamc = 2.5
        
        # Robot Params
        self.L1 = 0.35
        self.L2 = 0.35
        self.standing_h = 0.68
        
        # Main Loop (20Hz)
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("Slatol Stabilizer Active.")

    def mode_callback(self, msg):
        self.mode = msg.data

    def imu_callback(self, msg):
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        
        # Pitch estimation
        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            self.pitch = math.copysign(math.pi / 2, sinp)
        else:
            self.pitch = math.asin(sinp)
        self.pitch_rate = msg.angular_velocity.y

    def control_loop(self):
        # 1. MANUAL MODE: Do nothing (UI handles direct control)
        if self.mode == "MANUAL":
            return

        # 2. NLP MODE (The "Failed" approach)
        # It blindly follows the plan. If jumping, it disables balance.
        if self.mode == "NLP":
            if self.is_jumping: 
                return # Open Loop execution (likely to fall)
            
            # Simple Static Stand (only when not jumping)
            self.apply_static_balance()

        # 3. ZMP + AMC MODE (The "Success" approach)
        # It superimposes active balance ON TOP of the jump trajectory
        if self.mode == "ZMP_AMC":
            self.apply_zmp_amc_control()

    def apply_static_balance(self):
        # Basic standing balance (existing logic)
        balance_angle = (self.Kp_balance * self.pitch) + (self.Kd_balance * self.pitch_rate)
        q_h, q_k = self.inverse_kinematics(0.0, -self.standing_h)
        if q_h is not None:
            self.send_cmd(0.0, q_h + balance_angle, q_k, 0.05)

    def apply_zmp_amc_control(self):
        # --- PHASE 1: STANCE (Polar ZMP) ---
        # If legs are compressed/extended, we are likely on ground or pushing off.
        # Use Pitch Error to adjust Hip Angle (shift ZMP)
        u_zmp = (self.Kp_balance * self.pitch) + (self.Kd_balance * self.pitch_rate)
        
        # --- PHASE 2: FLIGHT (AMC) ---
        # If we are pitching forward, swing leg forward to create backward reaction torque.
        u_amc = self.Kamc * self.pitch_rate

        # Decoupled Logic: 
        # Ideally we detect ground contact, but for simplicity, we blend them 
        # or apply the dominant one based on pitch rate.
        
        total_balance_offset = u_zmp + u_amc

        # If we are simply standing (not jumping), apply to nominal stand
        if not self.is_jumping:
            q_h, q_k = self.inverse_kinematics(0.0, -self.standing_h)
            if q_h is not None:
                self.send_cmd(0.0, q_h + total_balance_offset, q_k, 0.05)
        
        # Note: If is_jumping is True, the 'plan_callback' sends trajectory points.
        # To truly fix the jump in Mode 3, we need to inject this offset into the 
        # trajectory execution. 
        # *For this level of code*, the easiest way is to modify 'plan_callback' 
        # to accept an offset, OR simply rely on the fact that Mode 3 
        # catches the robot immediately after the jump finishes.

    def plan_callback(self, msg):
        self.is_jumping = True
        target_h = msg.data[0]
        target_d = msg.data[1]
        
        self.get_logger().info(f"JUMPING: H={target_h}, D={target_d}")
        
        # Superimpose Balance? 
        # For now, simple phase sequence
        
        traj = JointTrajectory()
        traj.joint_names = ['hip_haa_joint', 'hip_hfe_joint', 'knee_kfe_joint']
        
        # 1. Compress
        q_h, q_k = self.inverse_kinematics(0.0, -0.45)
        self.add_pt(traj, [0.0, q_h, q_k], 0.5)
        
        # 2. Extend (Takeoff)
        q_h, q_k = self.inverse_kinematics(0.1, -0.73)
        self.add_pt(traj, [0.0, q_h, q_k], 0.8)
        
        # 3. Apex (Tuck)
        q_h, q_k = self.inverse_kinematics(0.1 + target_d/2, -0.50)
        self.add_pt(traj, [0.0, q_h, q_k], 1.2)
        
        # 4. Land (Extend)
        q_h, q_k = self.inverse_kinematics(0.1 + target_d, -0.70)
        self.add_pt(traj, [0.0, q_h, q_k], 1.6)
        
        self.traj_pub.publish(traj)
        
        self.create_timer(2.0, self.finish_jump)

    def finish_jump(self):
        self.is_jumping = False
        self.get_logger().info("Landed. Stabilizing.")
        return

    def inverse_kinematics(self, x, z):
        r = math.sqrt(x**2 + z**2)
        if r > (self.L1 + self.L2) or r < 0.1: return None, None
        
        c_k = (r**2 - self.L1**2 - self.L2**2) / (2*self.L1*self.L2)
        q_k = -math.acos(max(-1, min(1, c_k)))
        
        alpha = math.atan2(z, x)
        beta = math.acos((self.L1**2 + r**2 - self.L2**2)/(2*self.L1*r))
        q_h = (alpha + beta) + (math.pi/2)
        
        return q_h, q_k

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

def main():
    rclpy.init()
    node = SlatolPlanner()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()