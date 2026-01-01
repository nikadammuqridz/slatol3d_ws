import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray, Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tkinter as tk
import subprocess
import random
import time

class SlatolUI(Node):
    def __init__(self):
        super().__init__('slatol_ui')
        
        self.manual_pub = self.create_publisher(JointTrajectory, '/slatol_position_controller/joint_trajectory', 10)
        self.planner_pub = self.create_publisher(Float64MultiArray, '/slatol/planner/goal', 10) 
        self.mode_pub = self.create_publisher(String, '/slatol/mode', 10)
        self.standup_pub = self.create_publisher(Empty, '/slatol/cmd/standup', 10)

        self.root = tk.Tk()
        self.root.title("SLATOL Control Center")
        self.root.geometry("400x750") 
        self.root.attributes('-topmost', True) 

        self.mode = "PLANNER" 
        self.terrain_blocks = [] # Keep track of spawned blocks
        
        self.setup_ui()
        self.ui_timer = self.create_timer(0.1, self.update_gui)

    def setup_ui(self):
        tk.Label(self.root, text="SLATOL COMMANDER", font=("Arial", 14, "bold")).pack(pady=10)

        # --- 1. MODE SELECTION ---
        self.mode_frame = tk.LabelFrame(self.root, text="1. Control Mode", font=("Arial", 10, "bold"))
        self.mode_frame.pack(pady=5, fill="x", padx=20)
        
        tk.Button(self.mode_frame, text="MANUAL", bg="orange", width=8, 
                  command=lambda: self.set_mode("MANUAL")).grid(row=0, column=0, padx=5, pady=5)
        tk.Button(self.mode_frame, text="NLP", bg="gray", fg="white", width=8, 
                  command=lambda: self.set_mode("NLP")).grid(row=0, column=1, padx=5, pady=5)
        tk.Button(self.mode_frame, text="ZMP (Auto)", bg="green", fg="white", width=10, 
                  command=lambda: self.set_mode("ZMP_AMC")).grid(row=0, column=2, padx=5, pady=5)
        
        self.lbl_mode = tk.Label(self.root, text=f"Current: {self.mode}", fg="blue", font=("Arial", 11))
        self.lbl_mode.pack(pady=5)

        # --- 2. ENVIRONMENT & RESET ---
        self.sim_frame = tk.LabelFrame(self.root, text="2. Environment Tools", font=("Arial", 10, "bold"), fg="red")
        self.sim_frame.pack(pady=10, fill="x", padx=20)

        # NEW: Map Generator Button
        tk.Button(self.sim_frame, text="🎲 NEW RANDOM TERRAIN 🎲\n(Resets Robot & Map)", bg="blue", fg="white", font=("Arial", 10, "bold"), 
                  command=self.generate_new_map).pack(fill="x", pady=5, padx=5)

        tk.Button(self.sim_frame, text="RESPAWN ROBOT ONLY", bg="purple", fg="white", font=("Arial", 10), 
                  command=self.force_respawn).pack(fill="x", pady=5, padx=5)

        tk.Button(self.sim_frame, text="FULL SIM RESET", bg="red", fg="white", font=("Arial", 10), 
                  command=self.reset_sim).pack(fill="x", pady=5, padx=5)

        # --- 3. CONTROLS ---
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

        tk.Button(self.planner_frame, text="EXECUTE JUMP", bg="green", fg="white", font=("Arial", 12, "bold"),
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
            d = 0.0
            msg = Float64MultiArray()
            msg.data = [h, d]
            self.planner_pub.publish(msg)
        except ValueError: pass

    # --- IGNITION COMMAND HELPER ---
    def run_ign_cmd(self, cmd_base):
        # Tries specific world name first, then 'default', then 'empty'
        worlds = ["slatol_world", "default", "empty"] 
        for w in worlds:
            cmd = cmd_base.replace("WORLD_NAME", w)
            print(f"Trying: {w}")
            try:
                subprocess.Popen(cmd.split())
                return
            except Exception as e:
                print(f"Failed: {e}")

    # --- MAP GENERATION LOGIC ---
    def generate_new_map(self):
        print("\n--- GENERATING NEW TERRAIN ---")
        
        # 1. Reset Robot First
        self.force_respawn()
        
        # 2. Delete Old Blocks
        for i in range(10):
            name = f"block_{i}"
            cmd = f'ign service -s /world/WORLD_NAME/remove --reqtype ignition.msgs.Entity --reptype ignition.msgs.Boolean --timeout 2000 --req name:"{name}",type:MODEL'
            self.run_ign_cmd(cmd)
        
        # 3. Spawn New Random Blocks
        # We spawn 10 "Stepping Stones" forward
        for i in range(10):
            name = f"block_{i}"
            x = 0.5 + (i * 0.4) # Start 0.5m ahead, spaced 40cm
            z_var = random.uniform(-0.05, 0.05) # +/- 5cm height variation
            
            # Create SDF for a simple box
            sdf = f'''
            <sdf version="1.6">
                <model name="{name}">
                    <pose>{x} 0 {z_var} 0 0 0</pose>
                    <static>true</static>
                    <link name="link">
                        <collision name="col">
                            <geometry><box><size>0.4 1.0 0.1</size></box></geometry>
                            <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>
                        </collision>
                        <visual name="vis">
                            <geometry><box><size>0.4 1.0 0.1</size></box></geometry>
                            <material><ambient>0.5 0.3 0.1 1</ambient><diffuse>0.5 0.3 0.1 1</diffuse></material>
                        </visual>
                    </link>
                </model>
            </sdf>'''
            
            # Spawn Command
            cmd = f"ign service -s /world/WORLD_NAME/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 2000 --req sdf:'{sdf}'"
            self.run_ign_cmd(cmd)
            print(f"Spawned {name} at height {z_var:.2f}")

    def force_respawn(self):
        print("Respawning Robot...")
        # Reset Sliders
        self.slider_haa.set(0.0)
        self.slider_hfe.set(0.0)
        self.slider_kfe.set(0.0)
        
        # Teleport Robot
        cmd = 'ign service -s /world/WORLD_NAME/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 2000 --req name:"slatol",position:{x:0,y:0,z:0.65},orientation:{w:1,x:0,y:0,z:0}'
        self.run_ign_cmd(cmd)
        self.standup_pub.publish(Empty())

    def reset_sim(self):
        print("Resetting World...")
        self.slider_haa.set(0.0)
        self.slider_hfe.set(0.0)
        self.slider_kfe.set(0.0)
        cmd = 'ign service -s /world/WORLD_NAME/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 2000 --req reset:{all:true}'
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