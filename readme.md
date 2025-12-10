Here is the updated, concise, and correct content for your `README.md`. I have integrated the **Developer Setup** (VS Code + Multi-Git) and **Screen Recording** instructions directly into the flow so it serves as a complete guide for you and anyone else using the repo.

You can copy and paste the sections below directly into your `README.md` file.

-----

# SLATOL: Single-Legged Autonomous Takeoff and Landing Robot

**Author:** Nik Adam Muqridz Bin Abdul Hakham (2125501)
**Supervisor:** Dr. Hafiz Bin Iman
**Institution:** International Islamic University Malaysia (IIUM)

## 1\. Project Overview

This project focuses on the design and control of a Single-Legged Micro Aerial Vehicle (MAV) capable of autonomous takeoff and landing (SLATOL). It addresses dynamic instability during ground-to-air transitions using a 3-DOF robotic leg integrated with a fixed-wing MAV concept.

## 2\. Developer Environment Setup

> **⚠️ Important Reminder:** Follow these steps in order. This setup configures your environment so you can use the **VS Code "Sync Changes" button** to push to both GitHub and GitLab simultaneously without using the terminal every time.

### Phase A: OS & Tools Installation

1.  **Operating System:** Install **Ubuntu 22.04 LTS (Jammy Jellyfish)**.
2.  **IDE:** Install **Visual Studio Code (VS Code)**.
3.  **Recording Tool:** Install **Peek** (for creating GIF demos).
    ```bash
    sudo apt install peek
    ```

### Phase B: Configure Simultaneous Git Push (One-Time Setup)

Run these commands **once** in your terminal inside the repo folder (`~/slatol3d_ws/src/`) to link both repositories.

1.  **Initialize & Add Primary Remote (GitHub):**

    ```bash
    git init
    git remote add origin https://github.com/YOUR_USERNAME/slatol3d.git
    ```

2.  **Add Secondary Remote (GitLab):**

    ```bash
    git remote add gitlab https://gitlab.com/YOUR_USERNAME/slatol3d.git
    ```

3.  **Link Remotes for Simultaneous Push:**
    *This configures the 'origin' push URL to send data to BOTH links automatically.*

    ```bash
    git remote set-url --add --push origin https://github.com/YOUR_USERNAME/slatol3d.git
    git remote set-url --add --push origin https://gitlab.com/YOUR_USERNAME/slatol3d.git
    ```

    > **Verification:** Run `git remote -v`. You should see two `(push)` lines for `origin`. Now, simply click "Sync Changes" in VS Code to update both clouds\!

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

# Install Build Tools
sudo apt install python3-colcon-common-extensions -y
```

### Step 2: Build Workspace

```bash
cd ~/slatol3d_ws
colcon build --symlink-install
source install/setup.bash
```

> **Reminder:** Always run `source install/setup.bash` in every new terminal before running commands.

## 4\. Usage

### Launch Simulation

Spawns the SLATOL robot in Gazebo Fortress (Physics enabled).

```bash
ros2 launch slatol3d_bringup slatol.launch.py
```

### Manual Joint Control Test

Runs the Python script to command specific joint angles (HAA, HFE, KFE).

```bash
python3 src/slatol3d_bringup/test_control.py
```

## 5\. Demonstration & Documentation

To document your "Launch and Fall" tests effectively using **Peek**:

1.  **Open Peek:** Launch the app and position the green window over Gazebo.
2.  **Record:** Click **Record**, then run the `ros2 launch` or `test_control.py` command.
3.  **Save:** Stop recording and save the file as `demo_launch.gif`.
4.  **Organize:**
      * Create a `docs` folder: `mkdir -p src/slatol3d_bringup/docs`
      * Move the file: `mv demo_launch.gif src/slatol3d_bringup/docs/`
5.  **Embed:** Add `![Demo](docs/demo_launch.gif)` to this README to display it.

-----

### **Next Step**

Would you like me to verify if your `test_control.py` script correctly targets the controllers defined in your `slatol_controllers.yaml` file to ensure the demo runs smoothly?