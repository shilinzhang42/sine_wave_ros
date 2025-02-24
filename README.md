# sine_wave_ros
# **ROS 2 DevContainer Workspace**

## **📂 Directory Structure**
```bash
ws
├── .devcontainer
│   ├── devcontainer.json
│   └── Dockerfile
├── src
    └── package
```
- `.devcontainer/`: Used to configure **VS Code DevContainer**, containing `devcontainer.json` and `Dockerfile`.
- `src/`: Contains ROS 2 packages.

---

## **🚀 Create a ROS 2 Workspace**
If you don't have a workspace yet, follow these steps:
```bash
cd ~/
mkdir -p ws/src
cd ws
```
Then **open the `ws/` folder in VS Code**.

---

## **🔹 Using DevContainer**
### **1️ Ensure the following tools are installed**
- [VS Code](https://code.visualstudio.com/)
- [Docker](https://docs.docker.com/get-docker/)
- [Remote - Containers Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

## **2 Replace `zsl` with Your Username**
**If `devcontainer.json` or `Dockerfile` contains `zsl`, replace it with your Linux username**:
1. **Press `Ctrl + F` to search for `zsl`**
2. **Replace it with your username** (if unsure, run the following command to check):
   ```bash
   echo $USER
   ```

### **3 Open DevContainer in VS Code**
1. **Open the `ws/` folder**
2. **Press `Ctrl + Shift + P`**
3. Select **"Rebuild and Reopen in Container"**
4. Wait for the DevContainer to build.

---



## **4 Install Dependencies **
Inside the DevContainer, run:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd /home/ws/src
git clone https://github.com/PickNikRobotics/generate_parameter_library.git
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---