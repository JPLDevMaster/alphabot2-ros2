# ROS2 Drivers for Waveshare AlphaBot2-Pi

[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04%20LTS-orange)](https://releases.ubuntu.com/22.04/)
[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble%20Hawksbill-blue)](https://docs.ros.org/en/humble/index.html)

**🚧 Humble Migration & Network Automated Fork 🚧**

**Please Note:** This repository contains the core ROS2 packages for the Waveshare **Alphabot2-Pi** mobile robot. Originally developed for ROS2 Foxy by [Mik3Rizzo](https://github.com/Mik3Rizzo/), this fork has been upgraded to **ROS2 Humble** and features a fully automated installation script with an advanced Network Manager failover system (Lab Wi-Fi & Hotspot fallback) for field robotics.

## Overview

This repository provides the essential nodes to manage the AlphaBot2 sensors and actuators. The current implementation includes support for the **motors**, **IR obstacle sensors**, and the **Raspberry Pi camera** (using `v4l2_camera`). There is also an included node for **QR-code** detection.

## Packages

This repository currently includes the following core packages:

* **`alphabot2`**: The core nodes used to manage the AlphaBot2's hardware interface, sensors, and actuators.
* **`alphabot2_interfaces`**: Custom interfaces (ROS 2 messages) specifically designed for the AlphaBot2 ecosystem.

## Prerequisites

To use these packages, your system should meet the following **minimum** requirements:

* **Robot Hardware:** Waveshare Alphabot2-Pi mobile robot equipped with a Raspberry Pi 3, 1GB RAM version.
* **Operating System:** Ubuntu Server 22.04 LTS (Jammy Jellyfish) x64.

---

## 🛠️ System Pre-Installation (Critical)

As soon as you have flashed Ubuntu Server 22.04 onto your Raspberry Pi and booted it up, you **must** update the system packages before running any ROS 2 installation scripts.

Connect to your Pi and run:

```bash
sudo apt update
sudo apt full-upgrade -y
sudo reboot
```

---

## ⚡ Quick Installation (Automated)

We provide an "Ultimate" automated installer script that handles the ROS 2 Humble installation, Python dependencies, workspace compilation, camera legacy fixes, swap memory creation, and an advanced **Hotspot Fallback Network**. This script is called `install_alphabot2.sh` and can be downloaded seperatly from the repository (no need to clone it, the script will take care of that).

### 1. Run the Installer

After the system reboots from the pre-installation step, copy the `install_alphabot2.sh` script to your Pi, make it executable, and run it:

```bash
chmod +x install_alphabot2.sh
./install_alphabot2.sh
```

### 2. Installer Configuration Parameters

During execution, the script will prompt you for the following configuration details. Have them ready:

* **ROS2 Workspace Path:** Where to clone and build the code (Default: `~/alphabot2_ws`).
* **SWAP Size:** Dynamic swap creation to prevent RAM crashes during compilation (Default: `1GB`). It is advisable to use at least 500MB if you have a 1GB RAM Pi version.
* **Lab Wi-Fi Name (SSID) & Password:** The primary network the robot should connect to automatically every time it boots up.
* **Static IP:** A unique IP address required for this specific robot on the Lab/Work network (force static IP for easier ssh connections).
* **Gateway IP:** The router's gateway IP for the Lab/Work network.

### 3. Network Failover Logic

Once installed, the robot uses a dual-priority network system:

1. **Priority 1:** It will attempt to connect to your specified Lab/Work Wi-Fi using the Static IP.
2. **Priority 2 (Fallback):** If the Lab/Work Wi-Fi is unreachable (e.g., in the field), the robot will automatically create its own Access Point called **`AlphaBot2-Hotspot`** (Password: `alphabot2`). You can connect to this network and SSH into the robot at `10.42.0.1`. The creation of this automated hotspot may take up to 5 minutes, so please be patient.

---

## 📡 Remote PC Setup (ROS2 Communication)

To control the robot or view its topics (like the camera feed) from your personal computer over the network, your PC must be on the same Wi-Fi network (or connected to the robot's Hotspot).

Run the following commands on **your personal computer**'s terminal to enable ROS 2 network discovery:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

*(Tip: You can add these export lines to your PC's `~/.bashrc` file to make them permanent).*

---

## 🚀 Usage & Control

Once the installation is complete and the robot is booted, you can launch all the hardware nodes at once.

**1. Launch the Robot:**
SSH into the robot and run:

```bash
ros2 launch alphabot2 alphabot2_launch.py
```

**2. Move the Robot:**
From your remote PC (or a second terminal on the robot), publish a `Twist` message to the `alphabot2/cmd_vel` topic:

```bash
ros2 topic pub --rate 1 alphabot2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 5.0}}"
```

This command publishes a linear velocity of `1 m/s` and an angular rate of `5 rad/s` every second.

> **Hardware Limits Note:** The maximum theoretical linear velocity is `1.65 m/s` while the angular rate is `38.8 rad/s`. These quantities were estimated with the robot suspended from the floor, supplied with two Sony US14500VR Li-ion 3.7V batteries at full charge (resulting in a maximum wheel RPM of 750).

---

## About & Credits

* **Original Author:** Michele Rizzo, Master's Degree Computer Engineering student at University of Brescia. [GitHub: Mik3Rizzo](https://github.com/Mik3Rizzo/)
* **Humble Migration & Automated Setup:** João Penha Lopes, MSc Electrical and Computer Engineering student at IST - ULisboa, Researcher at Intitute For Systems and Robotics - Lisboa | [GitHub: JPLDevMaster](https://github.com/JPLDevMaster)

Please feel free to open an issue or submit a Pull Request if you encounter problems or want to contribute to the Humble migration! For ROS2 Foxy usage, please go check out the original repo available [here](https://github.com/Mik3Rizzo/alphabot2-ros2)!
