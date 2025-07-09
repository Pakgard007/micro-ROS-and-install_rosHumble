
![micro-ROS Logo](https://raw.githubusercontent.com/Pakgard007/micro-ROS-and-install_rosHumble/main/.images/microros_logo.png)

# 🚀 micro-ROS + ROS 2 Humble Installation Guide (Ubuntu 22.04)

This guide walks you through installing the **micro-ROS build system** and setting up a firmware workspace for microcontrollers using **ROS 2 Humble** on **Ubuntu 22.04**.

---

## 📑 Table of Contents

- [📋 Prerequisites](#-prerequisites)
- [✅ Step 1: Install ROS 2 Humble](#-step-1-install-ros-2-humble-if-not-installed)
- [✅ Step 2: Install micro-ROS Tools (`microros_ws`)](#-step-2-install-micro-ros-tools-microros_ws)
- [✅ Step 3: Create Firmware Workspace (`uros_ws`)](#-step-3-create-firmware-workspace-uros_ws)
- [🔧 Optional: Build and Flash Firmware](#-optional-build-and-flash-firmware)
- [🐳 Alternative: Using Docker](#-alternative-using-docker-not-recommended-for-flashing)
- [🗂️ Workspace Summary](#-workspace-summary)
- [🧪 Step 4: Verify Installation](#-step-4-verify-installation)

---

## 📋 Prerequisites

- ✅ Ubuntu 22.04 LTS
- ✅ ROS 2 Humble installed
- ✅ Python 3 and pip
- ✅ Internet connection

---

## ✅ Step 1: Install ROS 2 Humble (if not installed)

Follow the full official guide:  
📄 https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

Quick setup:

```bash
sudo apt update && sudo apt install curl gnupg lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions python3-rosdep python3-argcomplete

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS automatically in every terminal
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
````

---

## ✅ Step 2: Install micro-ROS Tools (`microros_ws`)

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Create workspace
mkdir -p ~/microros_ws/src
cd ~/microros_ws

# Clone micro-ROS setup tools (custom fork or original)
git clone https://github.com/Pakgard007/micro-ROS-and-install_rosHumble.git src/micro_ros_setup
# OR (official repo)
# git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Install dependencies
sudo apt update
rosdep install --from-paths src --ignore-src -y

# Ensure pip is installed
sudo apt install python3-pip

# Build workspace
colcon build
source install/local_setup.bash

# Automatically source on new terminals
grep -qxF "source ~/microros_ws/install/local_setup.bash" ~/.bashrc || echo "source ~/microros_ws/install/local_setup.bash" >> ~/.bashrc
```

---

## ✅ Step 3: Create Firmware Workspace (`uros_ws`)

```bash
# Create firmware workspace
mkdir ~/uros_ws
cd ~/uros_ws

# Example: Teensy + FreeRTOS
ros2 run micro_ros_setup create_firmware_ws.sh freertos teensy

# Other options:
# ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32
# ros2 run micro_ros_setup create_firmware_ws.sh nuttx olimex-stm32-e407
```

---

## 🔧 Optional: Build and Flash Firmware

```bash
# Build firmware
ros2 run micro_ros_setup build_firmware.sh

# Flash firmware (may require esptool.py, dfu-util, etc.)
ros2 run micro_ros_setup flash_firmware.sh
```

> ⚠️ Flashing requires appropriate drivers and permissions depending on your board.

---

## 🐳 Alternative: Using Docker (not recommended for flashing)

```bash
docker run -it --net=host -v /dev:/dev --privileged ros:humble
```

Then repeat **Step 2–3** inside the container.

---

## 🗂️ Workspace Summary

| Workspace     | Purpose                                     |
| ------------- | ------------------------------------------- |
| `microros_ws` | micro-ROS setup tools and scripts           |
| `uros_ws`     | microcontroller firmware building workspace |

---

## 🧪 Step 4: Verify Installation

To check if the tools are available:

```bash
ros2 run micro_ros_setup --help
```

You should see a list of available micro-ROS setup commands such as:

```
create_firmware_ws.sh
build_firmware.sh
flash_firmware.sh
```

---

หากคุณต้องการให้แปลงเป็นไฟล์ `.md` พร้อมใช้ หรือต้องการใส่ badge, โครงสร้างไฟล์เสริม หรือ CI ก็สามารถขอเพิ่มเติมได้เลยครับ ✅
```
