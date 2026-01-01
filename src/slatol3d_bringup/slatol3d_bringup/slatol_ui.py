import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray, Empty
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
        self.standup_pub = self.create_publisher(Empty, '/slatol/cmd/standup', 10)

        self.root = tk.Tk()
        self.root.title("SLATOL Control")
        self.root.geometry("380x680") 
        self.root.attributes('-topmost', True) 

        self.mode = "PLANNER" 
        self.setup_ui()
        self.ui_timer = self.create_timer(0.1, self.update_gui)

    def setup_ui(self):
        tk.Label(self.root, text="SLATOL COMMANDER", font=("Arial", 14, "bold")).pack(pady=10)

        # --- MODE SELECTION ---
        self.mode_frame = tk.LabelFrame(self.root, text="1. Select Control Mode", font=("Arial", 10, "bold"))
        self.mode_frame.pack(pady=5, fill="x", padx=20)
        
        tk.Button(self.mode_frame, text="MANUAL", bg="orange", width=10, 
                  command=lambda: self.set_mode("MANUAL")).grid(row=0, column=0, padx=5, pady=5)
        
        tk.Button(self.mode_frame, text="NLP (Fail)", bg="gray", fg="white", width=10, 
                  command=lambda: self.set_mode("NLP")).grid(row=0, column=1, padx=5, pady=5)
        
        tk.Button(self.mode_frame, text="ZMP (Success)", bg="green", fg="white", width=10, 
                  command=lambda: self.set_mode("ZMP_AMC")).grid(row=0, column=2, padx=5, pady=5)
        
        self.lbl_mode = tk.Label(self.root, text=f"Current: {self.mode}", fg="blue", font=("Arial", 11))
        self.lbl_mode.pack(pady=5)

        # --- SIMULATION TOOLS ---
        self.sim_frame = tk.LabelFrame(self.root, text="2. Recovery Tools", font=("Arial", 10, "bold"), fg="red")
        self.sim_frame.pack(pady=10, fill="x", padx=20)

        tk.Button(self.sim_frame, text="RESPAWN ROBOT (Teleport)", bg="purple", fg="white", font=("Arial", 10, "bold"), 
                  command=self.force_respawn).pack(fill="x", pady=5, padx=5)

        tk.Button(self.sim_frame, text="FULL WORLD RESET", bg="red", fg="white", font=("Arial", 10), 
                  command=self.reset_sim).pack(fill="x", pady=5, padx=5)

        # --- CONTROLS ---
        self.control_container = tk.Frame(self.root)
        self.control_container.pack(fill="both", expand=True)
        
        self.manual_frame = tk.LabelFrame(self.control_container, text="Manual Joint Control", font=("Arial", 10, "bold"))
        self.slider_haa = self.add_slider(self.manual_frame, "HAA", -0.5, 0.5)
        self.slider_hfe = self.add_slider(self.manual_frame, "HFE", -1.57, 1.57)
        self.slider_kfe = self.add_slider(self.manual_frame, "KFE", -2.0, 0.0)

        self.planner_frame = tk.LabelFrame(self.control_container, text="3. Jump Execution", font=("Arial", 10, "bold"))
        tk.Label(self.planner_frame, text="Takeoff H (m)").grid(row=0, column=0, padx=5, pady=5)
        self.entry_height = tk.Entry(self.planner_frame, width=8)
        self.entry_height.insert(0, "0.5") 
        self.entry_height.grid(row=0, column=1)

        tk.Label(self.planner_frame, text="Land Dist (m)").grid(row=1, column=0, padx=5, pady=5)
        self.entry_dist = tk.Entry(self.planner_frame, width=8)
        self.entry_dist.insert(0, "0.3")
        self.entry_dist.grid(row=1, column=1)

        tk.Button(self.planner_frame, text="EXECUTE JUMP", bg="blue", fg="white", font=("Arial", 12, "bold"),
                  command=self.send_plan).grid(row=3, column=0, columnspan=2, pady=10, sticky="ew")

        self.update_visibility()

    def add_slider(self, frame, label, min_val, max_val):
        tk.Label(frame, text=label).pack()
        s = tk.Scale(frame, from_=min_val, to=max_val, resolution=0.01, orient="horizontal", command=self.send_manual)
        s.pack(fill="x", padx=5)
        return s

    def set_mode(self, new_mode):
        self.mode = new_mode
        self.lbl_mode.config(text=f"Current: {self.mode}")
        self.update_visibility()
        msg = String()
        msg.data = self.mode
        self.mode_pub.publish(msg)

    def update_visibility(self):
        self.manual_frame.pack_forget()
        self.planner_frame.pack_forget()
        if self.mode == "MANUAL":
            self.manual_frame.pack(pady=10, padx=10, fill="both")
        else:
            self.planner_frame.pack(pady=10, padx=10, fill="both")

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

    def run_ign_cmd(self, cmd_base):
        # Helper to try multiple world names
        # Standard ignition/gazebo world names
        worlds = ["slatol_world", "default", "empty"] 
        for w in worlds:
            cmd = cmd_base.replace("WORLD_NAME", w)
            print(f"Trying command on world: {w}")
            try:
                # Use split to handle arguments correctly
                subprocess.Popen(cmd.split())
                return
            except Exception as e:
                print(f"Failed: {e}")

    def force_respawn(self):
        print("Respawning Robot...")
        # Template command - TIMEOUT INCREASED TO 10000 (10s)
        cmd = 'ign service -s /world/WORLD_NAME/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 10000 --req name:"slatol",position:{x:0,y:0,z:0.65},orientation:{w:1,x:0,y:0,z:0}'
        self.run_ign_cmd(cmd)
        self.standup_pub.publish(Empty())

    def reset_sim(self):
        print("Resetting World...")
        # TIMEOUT INCREASED TO 10000 (10s)
        cmd = 'ign service -s /world/WORLD_NAME/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 10000 --req reset:{all:true}'
        self.run_ign_cmd(cmd)

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