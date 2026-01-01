import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String, Empty
from sensor_msgs.msg import Imu
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class SlatolPlanner(Node):
    def __init__(self):
        super().__init__('slatol_planner')
        
        # --- ROS 2 INTERFACES ---
        self.create_subscription(Float64MultiArray, '/slatol/planner/goal', self.jump_command_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(String, '/slatol/mode', self.mode_callback, 10)
        self.create_subscription(Empty, '/slatol/cmd/standup', self.standup_callback, 10)
        self.traj_pub = self.create_publisher(JointTrajectory, '/slatol_position_controller/joint_trajectory', 10)
        
        # --- INTERNAL STATE ---
        self.mode = "PLANNER"
        self.state = "STANCE"
        
        # Orientation
        self.pitch = 0.0           # Theta (rad)
        self.pitch_rate = 0.0      # Theta_dot (rad/s)
        self.roll = 0.0
        
        # Jump Logic
        self.jump_start_time = 0.0
        self.flight_start_time = 0.0
        self.thrust_timer = None
        self.target_height = 0.5
        
        # --- CONTROL GAINS ---
        # ZMP Gain: Kp relates Pitch Error -> Hip Adjustment
        self.Kp_zmp = 0.8   
        self.Kd_zmp = 0.05  
        
        # AMC Gain
        # K_amc relates Body Spin -> Leg Kick Velocity
        self.K_amc = 1.2    
        
        # Geometry
        self.L1 = 0.35
        self.L2 = 0.35
        self.standing_h = 0.60 
        
        self.create_timer(0.01, self.control_loop) 
        self.get_logger().info("Slatol Logic Ready.")

    def jump_command_callback(self, msg):
        """Triggered when user clicks EXECUTE JUMP"""
        if self.mode == "MANUAL": return
        self.target_height = msg.data[0]
        
        if self.mode == "NLP":
            self.execute_nlp_trajectory()
        elif self.mode == "ZMP_AMC":
            if self.state == "FALLEN":
                self.get_logger().warn("Robot is FALLEN. Please Respawn.")
            else:
                self.start_zmp_jump()

    def control_loop(self):
        """100Hz Control Loop"""
        if self.mode != "ZMP_AMC": return

        now = self.get_clock().now().nanoseconds / 1e9
        
        # 1. Fall Detection
        if self.state != "FALLEN" and (abs(self.pitch) > 1.3 or abs(self.roll) > 1.3):
            self.get_logger().warn("FALL DETECTED. Disabling Control.")
            self.state = "FALLEN"
        
        if self.state == "FALLEN":
            return # Wait for Respawn
            
        elif self.state == "STANCE":
            # --- ZMP BALANCE ---
            # Moves Hip to keep GRF aligned with CoM
            balance_offset = (self.Kp_zmp * self.pitch) + (self.Kd_zmp * self.pitch_rate)
            
            # Prevent "Sinking": If balance needs crazy angle, clamp it
            balance_offset = max(-0.5, min(0.5, balance_offset))
            
            q_h, q_k = self.inverse_kinematics(0.0, -self.standing_h)
            if q_h is not None:
                self.send_cmd(0.0, q_h + balance_offset, q_k, 0.05)
                
        elif self.state == "THRUST":
            # --- LAUNCH ---
            t_elapsed = now - self.jump_start_time
            thrust_dur = 0.25
            
            progress = min(1.0, t_elapsed / thrust_dur)
            current_h = self.standing_h + (0.2 * progress)
            current_h = min(0.68, current_h) 
            
            q_h, q_k = self.inverse_kinematics(0.05, -current_h) 
            if q_h is not None:
                self.send_cmd(0.0, q_h, q_k, 0.01)
            
            if t_elapsed > thrust_dur:
                self.state = "FLIGHT"
                self.flight_start_time = now
                self.get_logger().info("LIFTOFF -> AMC ENGAGED")

        elif self.state == "FLIGHT":
            # --- AMC (Angular Momentum Control) ---
            # "Kick" leg to counter-spin body
            q_h_nom, q_k_nom = self.inverse_kinematics(0.1, -0.50) 
            amc_kick = self.K_amc * self.pitch_rate
            
            if q_h_nom is not None:
                self.send_cmd(0.0, q_h_nom + amc_kick, q_k_nom, 0.02)
            
            if (now - self.flight_start_time) > 0.4:
                self.state = "LANDING"

        elif self.state == "LANDING":
            # --- IMPACT ABSORPTION ---
            # Stiff ZMP response
            q_h, q_k = self.inverse_kinematics(0.15, -0.55)
            stabilizer = (self.Kp_zmp * 2.0) * self.pitch 
            
            if q_h is not None:
                self.send_cmd(0.0, q_h + stabilizer, q_k, 0.05)
            
            if (now - self.flight_start_time) > 1.5:
                self.state = "STANCE"
                self.get_logger().info("STABLE. Back to Stance.")

    def inverse_kinematics(self, x, z):
        """Standard 2-Link IK"""
        r = math.sqrt(x**2 + z**2)
        if r > (self.L1 + self.L2 - 0.01) or r < 0.1: return None, None
        
        c_k = (r**2 - self.L1**2 - self.L2**2) / (2*self.L1*self.L2)
        try:
            q_k = -math.acos(max(-1, min(1, c_k)))
            alpha = math.atan2(z, x)
            beta = math.acos((self.L1**2 + r**2 - self.L2**2)/(2*self.L1*r))
            q_h = (alpha + beta) + (math.pi/2)
            return q_h, q_k
        except ValueError: return None, None

    def start_zmp_jump(self):
        if self.state != "STANCE": return
        self.get_logger().info("JUMP SEQUENCE STARTED")
        q_h, q_k = self.inverse_kinematics(0.0, -0.45)
        if q_h is not None:
            self.send_cmd(0.0, q_h, q_k, 0.5)
        self.thrust_timer = self.create_timer(0.5, self.trigger_thrust)

    def trigger_thrust(self):
        if self.thrust_timer:
            self.thrust_timer.cancel()
            self.thrust_timer = None
        self.state = "THRUST"
        self.jump_start_time = self.get_clock().now().nanoseconds / 1e9

    def execute_nlp_trajectory(self):
        # Open Loop (For Comparison)
        traj = JointTrajectory()
        traj.joint_names = ['hip_haa_joint', 'hip_hfe_joint', 'knee_kfe_joint']
        steps = [(0.5, 0.0, -0.45), (0.8, 0.1, -0.68), (1.2, 0.2, -0.50), (1.6, 0.3, -0.65)]
        for t, x, z in steps:
            q_h, q_k = self.inverse_kinematics(x, z)
            if q_h: self.add_pt(traj, [0.0, q_h, q_k], t)
        self.traj_pub.publish(traj)

    def standup_callback(self, msg):
        self.state = "STANCE"
        self.send_cmd(0.0, 0.0, 0.0, 1.0)

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
        self.roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    
    def mode_callback(self, msg):
        self.mode = msg.data
        if self.mode == "ZMP_AMC" and self.state == "FALLEN" and abs(self.pitch) < 0.5:
             self.state = "STANCE"

def main():
    rclpy.init()
    node = SlatolPlanner()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()