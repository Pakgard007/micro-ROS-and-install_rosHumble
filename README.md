
---

# 🛠️ micro-ROS Build System Setup for ROS 2 Humble (Ubuntu 22.04)

## 📋 Prerequisites

* Ubuntu 22.04 LTS
* ROS 2 Humble installed
* Python 3 and pip
* Internet connection

---

## ✅ Step 1: Install ROS 2 Humble (if not installed)

Follow the full guide here: [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

Quick install:

```bash
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop

# Source ROS automatically in every terminal
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## ✅ Step 2: Install micro-ROS Tools (`microros_ws`)

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Create workspace for micro-ROS tools
mkdir -p ~/microros_ws/src
cd ~/microros_ws

# Clone the micro_ros_setup repository
git clone https://github.com/Pakgard007/micro-ROS-and-install_rosHumble.git src/micro_ros_setup



# Install dependencies
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -y

# Install pip (if not installed)
sudo apt install python3-pip

# Build the workspace
colcon build
source install/local_setup.bash

# Add to .bashrc for automatic sourcing
echo "source ~/microros_ws/install/local_setup.bash" >> ~/.bashrc
```

---

## ✅ Step 3: Create Firmware Workspace (`uros_ws`)

```bash
# Create firmware workspace
mkdir ~/uros_ws
cd ~/uros_ws

# Create firmware environment for your target board

# Example for Teensy + FreeRTOS:
ros2 run micro_ros_setup create_firmware_ws.sh freertos teensy

# Other examples:
# ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32
# ros2 run micro_ros_setup create_firmware_ws.sh nuttx olimex-stm32-e407
```

---

## 🔧 Optional: Build and Flash Firmware

```bash
# Build the firmware
ros2 run micro_ros_setup build_firmware.sh

# Flash it to the board
ros2 run micro_ros_setup flash_firmware.sh
```

> ⚠️ The flash command may require extra drivers or toolchains depending on your board (e.g. `esptool.py`, `dfu-util`, etc.)

---

## 🐳 \[Alternative] Using Docker (not recommended for flashing)

```bash
docker run -it --net=host -v /dev:/dev --privileged ros:humble
```

Then follow Steps 2–3 inside the container.

---

## 🗂️ Workspace Summary

| Workspace     | Purpose                                      |
| ------------- | -------------------------------------------- |
| `microros_ws` | Installs the micro-ROS toolchain and scripts |
| `uros_ws`     | Used for creating and building firmware      |

---

