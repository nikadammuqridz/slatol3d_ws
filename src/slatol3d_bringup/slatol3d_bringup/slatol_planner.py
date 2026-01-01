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
        
        self.create_subscription(Float64MultiArray, '/slatol/planner/goal', self.jump_command_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(String, '/slatol/mode', self.mode_callback, 10)
        self.create_subscription(Empty, '/slatol/cmd/standup', self.standup_callback, 10)
        
        self.traj_pub = self.create_publisher(JointTrajectory, '/slatol_position_controller/joint_trajectory', 10)
        
        self.mode = "PLANNER"
        self.pitch = 0.0
        self.pitch_rate = 0.0
        self.roll = 0.0
        
        # States: STANCE, THRUST, FLIGHT, LANDING, FALLEN
        self.state = "STANCE"
        self.jump_start_time = 0.0
        self.flight_start_time = 0.0
        self.thrust_timer = None # Handle for the timer
        
        # Gains
        self.Kp_zmp = 0.8
        self.Kd_zmp = 0.05
        self.K_amc = 1.2    
        
        # Geometry
        self.L1 = 0.35
        self.L2 = 0.35
        self.standing_h = 0.60 
        
        self.create_timer(0.01, self.control_loop)
        self.get_logger().info("Slatol Stabilizer Ready (Fixed Timer).")

    def mode_callback(self, msg):
        self.mode = msg.data
        self.get_logger().info(f"Mode changed to: {self.mode}")
        if self.mode == "ZMP_AMC" and self.state == "FALLEN":
             if abs(self.pitch) < 0.5:
                 self.state = "STANCE"

    def standup_callback(self, msg):
        self.get_logger().info("RESPAWN CONFIRMED: Resetting Planner State")
        self.state = "STANCE"
        self.send_cmd(0.0, 0.0, 0.0, 1.0) # Reset joints slowly

    def imu_callback(self, msg):
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        sinp = 2.0 * (qw * qy - qz * qx)
        self.pitch = math.asin(max(-1.0, min(1.0, sinp)))
        self.pitch_rate = msg.angular_velocity.y
        sinr = 2.0 * (qw * qx + qy * qz)
        cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
        self.roll = math.atan2(sinr, cosr)

    def jump_command_callback(self, msg):
        if self.mode == "MANUAL": return
        self.target_height = msg.data[0]
        if self.mode == "NLP":
            self.execute_nlp_trajectory()
        elif self.mode == "ZMP_AMC":
            if self.state == "FALLEN":
                self.get_logger().warn("Robot is FALLEN. Please Respawn first.")
            else:
                self.start_zmp_jump()

    def control_loop(self):
        if self.mode != "ZMP_AMC": return

        now = self.get_clock().now().nanoseconds / 1e9
        
        if self.state != "FALLEN":
            if abs(self.pitch) > 1.3 or abs(self.roll) > 1.3:
                self.get_logger().warn("ROBOT FALLEN! Disabling Control.")
                self.state = "FALLEN"
        
        if self.state == "FALLEN":
            return 
            
        elif self.state == "STANCE":
            balance_comp = (self.Kp_zmp * self.pitch) + (self.Kd_zmp * self.pitch_rate)
            q_h, q_k = self.inverse_kinematics(0.0, -self.standing_h)
            if q_h is not None:
                self.send_cmd(0.0, q_h + balance_comp, q_k, 0.05)
                
        elif self.state == "THRUST":
            t_elapsed = now - self.jump_start_time
            thrust_dur = 0.25
            
            progress = min(1.0, t_elapsed / thrust_dur)
            max_ext = 0.68 
            target_ext = self.standing_h + (0.2 * progress)
            current_h = min(max_ext, target_ext)
            
            q_h, q_k = self.inverse_kinematics(0.05, -current_h) 
            if q_h is not None:
                self.send_cmd(0.0, q_h, q_k, 0.01)
            
            if t_elapsed > thrust_dur:
                self.state = "FLIGHT"
                self.flight_start_time = now
                self.get_logger().info("LIFTOFF! Engaging AMC.")

        elif self.state == "FLIGHT":
            q_h_nom, q_k_nom = self.inverse_kinematics(0.1, -0.50) 
            amc_kick = self.K_amc * self.pitch_rate
            if q_h_nom is not None:
                self.send_cmd(0.0, q_h_nom + amc_kick, q_k_nom, 0.02)
            
            if (now - self.flight_start_time) > 0.4:
                self.state = "LANDING"

        elif self.state == "LANDING":
            q_h, q_k = self.inverse_kinematics(0.15, -0.55)
            balance_comp = (self.Kp_zmp * 2.0) * self.pitch 
            if q_h is not None:
                self.send_cmd(0.0, q_h + balance_comp, q_k, 0.05)
            
            if (now - self.flight_start_time) > 1.5:
                self.state = "STANCE"
                self.get_logger().info("Touchdown. Stabilizing.")

    def start_zmp_jump(self):
        if self.state != "STANCE": return # Prevent double trigger
        
        self.get_logger().info("COMPRESSING...")
        q_h, q_k = self.inverse_kinematics(0.0, -0.45)
        if q_h is not None:
            self.send_cmd(0.0, q_h, q_k, 0.5)
        
        # Schedule Thrust (Store timer to cancel it later!)
        self.thrust_timer = self.create_timer(0.5, self.trigger_thrust)
        
    def trigger_thrust(self):
        # --- CRITICAL FIX: CANCEL THE TIMER ---
        if self.thrust_timer:
            self.thrust_timer.cancel()
            self.thrust_timer.destroy()
            self.thrust_timer = None
        # --------------------------------------
        
        self.get_logger().info("THRUSTING!")
        self.state = "THRUST"
        self.jump_start_time = self.get_clock().now().nanoseconds / 1e9

    def execute_nlp_trajectory(self):
        self.get_logger().info("EXECUTING NLP (OPEN LOOP)")
        traj = JointTrajectory()
        traj.joint_names = ['hip_haa_joint', 'hip_hfe_joint', 'knee_kfe_joint']
        steps = [(0.5, 0.0, -0.45), (0.8, 0.1, -0.68), (1.2, 0.2, -0.50), (1.6, 0.3, -0.65)]
        for t, x, z in steps:
            q_h, q_k = self.inverse_kinematics(x, z)
            if q_h: self.add_pt(traj, [0.0, q_h, q_k], t)
        self.traj_pub.publish(traj)

    def inverse_kinematics(self, x, z):
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

    def send_cmd(self, haa, hfe, kfe, dt):
        if haa is None: return
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