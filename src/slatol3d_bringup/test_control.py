import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class JointCommander(Node):
    def __init__(self):
        super().__init__('joint_commander')
        # Topic must match the controller defined in slatol_controllers.yaml
        self.pub = self.create_publisher(JointTrajectory, '/slatol_position_controller/joint_trajectory', 10)
        self.get_logger().info('Commander Node Started. Moving robot in 2 seconds...')
        self.timer = self.create_timer(2.0, self.move_robot)

    def move_robot(self):
        msg = JointTrajectory()
        msg.joint_names = ['hip_haa_joint', 'hip_hfe_joint', 'knee_kfe_joint']
        point = JointTrajectoryPoint()
        # Moving joints to: 0.5 rad, 1.0 rad, -1.0 rad
        point.positions = [0.5, 1.0, -1.0] 
        point.time_from_start = Duration(sec=2)
        msg.points.append(point)
        
        self.pub.publish(msg)
        self.get_logger().info('Command Sent: [0.5, 1.0, -1.0]')
        
        # FIX: Pass 'self.timer' to the function
        self.destroy_timer(self.timer) 

def main():
    rclpy.init()
    node = JointCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()