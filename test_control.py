import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class JointCommander(Node):
    def __init__(self):
        super().__init__('joint_commander')
        self.publisher_ = self.create_publisher(JointTrajectory, 
            '/slatol_position_controller/joint_trajectory', 10)
        
    def send_command(self, haa_pos, hfe_pos, kfe_pos, duration_sec):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['hip_haa_joint', 'hip_hfe_joint', 'knee_kfe_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [haa_pos, hfe_pos, kfe_pos]
        point.velocities = [0.0, 0.0, 0.0]
        point.accelerations = [0.0, 0.0, 0.0]
        point.time_from_start = Duration(sec=duration_sec, nanosec=0)
        
        msg.points.append(point)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Command sent: HAA={haa_pos}, HFE={hfe_pos}, KFE={kfe_pos}')

def manual_control_test():
    rclpy.init()
    node = JointCommander()
    node.get_logger().info('Starting control test...')
    
    time.sleep(2.0) 

    # 1. Command the robot to an angled position (should move joints)
    node.send_command(0.5, 1.0, -1.5, 1) 
    time.sleep(1.5) 
    
    # 2. Command the robot back to a centered position
    node.send_command(0.0, 0.0, 0.0, 1)
    time.sleep(1.5)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    manual_control_test()