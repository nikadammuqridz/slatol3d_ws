Here is the compiled **README.md** content and the specific setup instructions for your multi-git workflow and screen recording.

### **Part 1: The README.md File**

Create a file named `README.md` in the root of your repository (`~/slatol3d_ws/src/slatol3d_bringup/README.md` or the workspace root) and paste this content.

-----

# SLATOL: Single-Legged Autonomous Takeoff and Landing Robot

**Author:** Nik Adam Muqridz Bin Abdul Hakham (2125501)  
**Supervisor:** Dr. Hafiz Bin Iman  
**Institution:** International Islamic University Malaysia (IIUM)

## 1\. Project Overview

This project focuses on the design and control of a Single-Legged Micro Aerial Vehicle (MAV) capable of autonomous takeoff and landing (SLATOL). It addresses dynamic instability during ground-to-air transitions using a 3-DOF robotic leg integrated with a fixed-wing MAV concept.

## 2\. Prerequisites

  * **OS:** Ubuntu 22.04 LTS (Jammy Jellyfish)
  * **ROS 2:** Humble Hawksbill
  * **Simulator:** Gazebo Fortress

## 3\. Installation Guide

### Step 1: Environment Setup

```bash
# Install ROS 2 Humble
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop -y

# Install Gazebo Fortress & Bridge
sudo apt install ros-humble-ros-gz ros-humble-ign-ros2-control -y

# Install Build Tools & GUI Libs
sudo apt install python3-colcon-common-extensions python3-tk -y
```

### Step 2: Build Workspace

```bash
cd ~/slatol3d_ws
colcon build --symlink-install
source install/setup.bash
```

## 4\. Usage

### Launch Simulation

Spawns the SLATOL robot in Gazebo Fortress (Physics enabled).

```bash
ros2 launch slatol3d_bringup slatol.launch.py
```

### Manual Joint Control Test

Runs a script to command specific joint angles (HAA, HFE, KFE).

```bash
python3 src/slatol3d_bringup/test_control.py
```

### GUI Control Interface

Launches a graphical interface for Manual Control and Inverse Kinematics (IK) Planner.

```bash
ros2 run slatol3d_bringup gui
```

## 5\. Demonstration

**Successful Launch & Physics Test:**
*(Robot spawns, gravity applies, and joints respond to commands)*

-----

### **Part 2: Setup VS Code for Simultaneous GitHub & GitLab**

To push to **both** GitHub and GitLab automatically when you click "Sync/Push" in VS Code, follow these terminal commands inside your repository folder:

1.  **Initialize Git (if not done):**

    ```bash
    cd ~/slatol3d_ws/src/
    git init
    git add .
    git commit -m "Initial commit of SLATOL project"
    ```

2.  **Add Your Remotes:**

    ```bash
    # Add GitHub as the primary 'origin'
    git remote add origin https://github.com/YOUR_USERNAME/slatol3d.git

    # Add GitLab as a secondary remote
    git remote add gitlab https://gitlab.com/YOUR_USERNAME/slatol3d.git
    ```

3.  **Configure "Simultaneous" Push:**
    This command tells git that when you push to `origin`, it should push to **both** URLs.

    ```bash
    git remote set-url --add --push origin https://github.com/YOUR_USERNAME/slatol3d.git
    git remote set-url --add --push origin https://gitlab.com/YOUR_USERNAME/slatol3d.git
    ```

4.  **Verify:**
    Run `git remote -v`. You should see two `(push)` lines for `origin`. Now, when you use Source Control in VS Code, it updates both repositories\!

-----

### **Part 3: Adding the Screen Recording**

The best way to show a "Launch and Fall" test in a README is using a **GIF**, as it auto-plays and works on all browsers.

1.  **Record the Screen:**

      * **Install Peek** (Simple GIF recorder for Ubuntu):
        ```bash
        sudo apt install peek
        ```
      * Open `ros2 launch...` and Gazebo.
      * Open Peek, place the window over Gazebo, and click **Record**.
      * Run your `test_control.py` script so the robot moves.
      * Stop recording and save as `demo_launch.gif`.

2.  **Add to Repo:**

      * Create a folder named `docs` inside your package.
      * Move the GIF there: `mv demo_launch.gif src/slatol3d_bringup/docs/`
      * The `README.md` code provided above already links to this path (`docs/demo_launch.gif`).

3.  **Push:**

      * Commit the GIF file and the README.
      * Push to your repos.
