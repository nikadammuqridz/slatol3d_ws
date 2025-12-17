import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tkinter as tk
import subprocess

class SlatolUI(Node):
    def __init__(self):
        super().__init__('slatol_ui')
        
        self.manual_pub = self.create_publisher(JointTrajectory, '/slatol_position_controller/joint_trajectory', 10)
        self.planner_pub = self.create_publisher(Float64MultiArray, '/slatol/planner/goal', 10) 
        self.mode_pub = self.create_publisher(String, '/slatol/mode', 10)

        self.root = tk.Tk()
        self.root.title("SLATOL Control")
        self.root.geometry("320x550")
        self.root.attributes('-topmost', True) 

        self.mode = "PLANNER" 
        self.setup_ui()
        self.ui_timer = self.create_timer(0.1, self.update_gui)

    def setup_ui(self):
        tk.Label(self.root, text="SLATOL COMMANDER", font=("Arial", 14, "bold")).pack(pady=10)

        # Mode Switch
        self.mode_btn = tk.Button(self.root, text=f"Mode: {self.mode}", bg="green", fg="white", font=("Arial", 10, "bold"), command=self.toggle_mode)
        self.mode_btn.pack(pady=5, fill="x", padx=20)

        # MANUAL FRAME
        self.manual_frame = tk.LabelFrame(self.root, text="Manual Joint Control", font=("Arial", 10, "bold"))
        self.slider_haa = self.add_slider(self.manual_frame, "HAA", -0.5, 0.5)
        self.slider_hfe = self.add_slider(self.manual_frame, "HFE", -1.57, 1.57)
        self.slider_kfe = self.add_slider(self.manual_frame, "KFE", -2.0, 0.0)

        # PLANNER FRAME
        self.planner_frame = tk.LabelFrame(self.root, text="Winkler Planner (NLP)", font=("Arial", 10, "bold"))
        tk.Label(self.planner_frame, text="Takeoff Height (m)").grid(row=0, column=0, padx=5, pady=5)
        self.entry_height = tk.Entry(self.planner_frame, width=8)
        self.entry_height.insert(0, "0.5")
        self.entry_height.grid(row=0, column=1)

        tk.Label(self.planner_frame, text="Land Dist (m)").grid(row=1, column=0, padx=5, pady=5)
        self.entry_dist = tk.Entry(self.planner_frame, width=8)
        self.entry_dist.insert(0, "0.3")
        self.entry_dist.grid(row=1, column=1)

        tk.Button(self.planner_frame, text="EXECUTE", bg="blue", fg="white", command=self.send_plan).grid(row=3, column=0, columnspan=2, pady=10, sticky="ew")

        # RESET
        tk.Button(self.root, text="RESET ROBOT POSE", bg="red", fg="white", font=("Arial", 10, "bold"), command=self.reset_sim).pack(side="bottom", fill="x", pady=20, padx=20)

        self.update_visibility()

    def add_slider(self, frame, label, min_val, max_val):
        tk.Label(frame, text=label).pack()
        s = tk.Scale(frame, from_=min_val, to=max_val, resolution=0.01, orient="horizontal", command=self.send_manual)
        s.pack(fill="x", padx=5)
        return s

    def toggle_mode(self):
        self.mode = "PLANNER" if self.mode == "MANUAL" else "MANUAL"
        self.mode_btn.config(text=f"Mode: {self.mode}", bg="green" if self.mode == "PLANNER" else "orange")
        self.update_visibility()
        msg = String()
        msg.data = self.mode
        self.mode_pub.publish(msg)

    def update_visibility(self):
        if self.mode == "PLANNER":
            self.manual_frame.pack_forget()
            self.planner_frame.pack(pady=10, padx=10, fill="both")
        else:
            self.planner_frame.pack_forget()
            self.manual_frame.pack(pady=10, padx=10, fill="both")

    def send_manual(self, val):
        if self.mode != "MANUAL": return
        msg = JointTrajectory()
        msg.joint_names = ['hip_haa_joint', 'hip_hfe_joint', 'knee_kfe_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [self.slider_haa.get(), self.slider_hfe.get(), self.slider_kfe.get()]
        pt.time_from_start = Duration(sec=0, nanosec=50000000)
        msg.points.append(pt)
        self.manual_pub.publish(msg)

    def send_plan(self):
        try:
            h = float(self.entry_height.get())
            d = float(self.entry_dist.get())
            msg = Float64MultiArray()
            msg.data = [h, d]
            self.planner_pub.publish(msg)
        except ValueError: pass

    def reset_sim(self):
        # 1. Teleport to Z=0.8 (Safely above ground so it falls freely)
        subprocess.Popen([
            "ign", "service", "-s", "/world/slatol_world/set_pose",
            "--reqtype", "ignition.msgs.Pose",
            "--req", 'name: "slatol", position: {x: 0, y: 0, z: 0.8}, orientation: {x: 0, y: 0, z: 0, w: 1}'
        ])
        
        # 2. Reset Joints to Zero (Vertical)
        msg = JointTrajectory()
        msg.joint_names = ['hip_haa_joint', 'hip_hfe_joint', 'knee_kfe_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [0.0, 0.0, 0.0]
        pt.time_from_start = Duration(sec=0, nanosec=100000000)
        msg.points.append(pt)
        
        # Force publish to ensure reset happens even in Planner mode
        self.manual_pub.publish(msg)
        self.manual_pub.publish(msg)

    def update_gui(self):
        self.root.update()

def main():
    rclpy.init()
    node = SlatolUI()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()