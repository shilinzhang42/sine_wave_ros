# **Sine Wave**
---

## **ROS 2 DevContainer Environment Setup**

### **1 Clone the Repository**
```bash
git clone https://github.com/shilinzhang42/sine_wave_ros.git
```

### **2 Create a ROS 2 Workspace**
If you don't have a workspace yet, follow these steps:
```bash
cd ~/
mkdir -p ws/src
cd ws
```
Then **open the `ws/` folder in VS Code**.


### 3 **Directory Structure of Workspace**
```bash
ws
├── .devcontainer
│   ├── devcontainer.json
│   └── Dockerfile
├── src
    └── sine_wave_ros
```
- `.devcontainer/`: move the files in sine_wave_ros/container to ws/.devcontainer
- `src/`: contains ROS 2 packages.

### **4 Ensure the following tools are installed**
- [VS Code](https://code.visualstudio.com/)
- [Docker](https://docs.docker.com/get-docker/)
- [Remote - Containers Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

### **5 Replace `zsl` with Your Username**
**If `devcontainer.json` or `Dockerfile` contains `zsl`, replace it with your Linux username**:
1. **Press `Ctrl + F` to search for `zsl`**
2. **Replace it with your username** (if unsure, run the following command to check):
   ```bash
   echo $USER
   ```

### **6 Open DevContainer in VS Code**
1. **Open the `ws/` folder**
2. **Press `Ctrl + Shift + P`**
3. Select **"Rebuild and Reopen in Container"**
4. Wait for the DevContainer to build.



### **7 Install Dependencies**
Inside the DevContainer, run:
```bash
sudo apt update
sudo apt install python3-pandas python3-matplotlib -y
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
cd /home/ws/src
git clone https://github.com/PickNikRobotics/generate_parameter_library.git
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```
### **8 Build the Package**
```bash
colcon build --packages-select sine_wave_package
source install/setup.bash
```
---

## **Run the Pragramm**

### **1 Run the Publisher and Subscriber**

```bash
cd /home/ws
source install/setup.bash
ros2 launch sine_wave_package sine_wave_launch.py
```

### **2 Visualize the Sine Wave**

```bash
cd /home/ws
python3 src/sine_wave_ros/plot/plot.py
```

The image will be saved in `src/sine_wave_ros/plot/sine_wave_plot.png`