import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import numpy as np

class SlatolPlanner(Node):
    def __init__(self):
        super().__init__('slatol_planner')
        
        # Subs
        self.create_subscription(Float64MultiArray, '/slatol/planner/goal', self.plan_callback, 10)
        
        # Pubs
        self.traj_pub = self.create_publisher(JointTrajectory, '/slatol_position_controller/joint_trajectory', 10)
        
        # Robot Params (L1=L2=0.35m)
        self.L1 = 0.35
        self.L2 = 0.35
        
        self.get_logger().info("Winkler Phase-Based Planner Node Started")

    def plan_callback(self, msg):
        target_h = msg.data[0]
        target_d = msg.data[1]
        
        self.get_logger().info(f"Computing Trajectory for H={target_h}, D={target_d}...")
        
        # --- WINKLER 2018 (Simplified) ---
        # Instead of full NLP, we generate a Phase-Based Spline for the CoM
        # Phases: Stance (Compression) -> Flight (Ballistic) -> Landing (Impact)
        
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['hip_haa_joint', 'hip_hfe_joint', 'knee_kfe_joint']
        
        # Generate Waypoints (CoM Space -> Joint Space)
        # 1. Compress (Down)
        # 2. Extend (Up - Takeoff)
        # 3. Retract (Flight Apex)
        # 4. Extend (Landing)
        
        # Simplified time-steps
        times = [0.5, 1.0, 1.5, 2.0] 
        
        # Cartesian Targets [x, z] relative to hip
        # Z must be negative relative to hip
        waypoints_cartesian = [
            [0.0, -0.45],       # Compress
            [0.1, -0.65],       # Takeoff Extension
            [0.1 + target_d/2, -0.3], # Flight (Retract legs)
            [0.1 + target_d, -0.60]   # Landing Extension
        ]
        
        for i, (x, z) in enumerate(waypoints_cartesian):
            q_haa = 0.0 # Planar
            q_hfe, q_kfe = self.inverse_kinematics(x, z)
            
            if q_hfe is None:
                self.get_logger().error(f"Target unreachable: {x}, {z}")
                continue
                
            pt = JointTrajectoryPoint()
            pt.positions = [q_haa, q_hfe, q_kfe]
            pt.time_from_start = Duration(sec=int(times[i]), nanosec=int((times[i]%1)*1e9))
            traj_msg.points.append(pt)
            
        self.traj_pub.publish(traj_msg)
        self.get_logger().info("Trajectory Published")

    def inverse_kinematics(self, x, z):
        # 2-Link Planar IK
        # Hip at (0,0). Foot at (x,z).
        # Cosine Rule
        r = math.sqrt(x**2 + z**2)
        
        # Check Reachability
        if r > (self.L1 + self.L2) or r == 0:
            return None, None
            
        # Knee Angle (q2)
        # c2 = (x^2 + z^2 - L1^2 - L2^2) / (2*L1*L2)
        c2 = (x**2 + z**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        q_kfe = -math.acos(c2) # Knee backwards (bird-like) typically or forward? 
                               # Slatol URDF limits knee -2.5 to 0 (Backwards bending)
        
        # Hip Angle (q1)
        # theta = atan2(z, x)
        # alpha = atan2(L2*sin(q2), L1 + L2*cos(q2))
        # q1 = theta - alpha
        
        # Adjust for robot orientation (Z down vs Z up)
        # In URDF: Thigh rotates Y. Z is down. 
        # Simple Trig:
        alpha = math.atan2(self.L2 * math.sin(q_kfe), self.L1 + self.L2 * math.cos(q_kfe))
        theta = math.atan2(z, x) # Z is negative usually
        
        # In URDF, 0 is straight down? 
        # No, Cylinder origin is center. 
        # Standard analytical IK:
        q_hfe = (theta - alpha) - (-1.57) # Offset to align with vertical being -1.57 (approx)
        
        # Corrections for Gazebo Frame
        # Let's assume standard IK frame: 0 is X-axis. 
        # Robot HFE 0 is horizontal? URDF says limit -1.57 to 1.57.
        # If we assume 0 is straight down:
        q_hfe = theta - alpha + 1.57 # Rotate 90deg
        
        return q_hfe, q_kfe

def main():
    rclpy.init()
    node = SlatolPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()