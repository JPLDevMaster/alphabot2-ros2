#!/usr/bin/env bash

set -e

# ----------------------------------------------------------------------
# Helper Variables for Formatting
# ----------------------------------------------------------------------
BOLD="\e[1m"
RESET="\e[0m"
GREEN="\e[32m"
BLUE="\e[34m"
RED="\e[31m"

echo -e "${BLUE}----------------------------------------------------------------------${RESET}"
echo -e "${BOLD}[AlphaBot2]: Starting ROS2 Humble Installation for Waveshare Alphabot2-Pi${RESET}"
echo -e "${BLUE}----------------------------------------------------------------------${RESET}"
echo ""

# ----------------------------------------------------------------------
# [0] User Configuration
# ----------------------------------------------------------------------
DEFAULT_WS=~/alphabot2_ws
read -p "[AlphaBot2]: Enter the path for your ROS2 workspace (Default: $DEFAULT_WS): " USER_WS_INPUT
WORKSPACE_DIR=${USER_WS_INPUT:-$DEFAULT_WS}

DEFAULT_SWAP="1GB"
read -p "[AlphaBot2]: Enter SWAP size (e.g., 500MB, 2GB, 4GB) or '0' to skip (Default: $DEFAULT_SWAP): " USER_SWAP_INPUT
SWAP_SIZE=${USER_SWAP_INPUT:-$DEFAULT_SWAP}

echo ""
echo -e "${YELLOW}We will configure a PRIMARY connection (Lab) and a BACKUP Hotspot.${RESET}"

TARGET_SSID=""
while [[ -z "$TARGET_SSID" ]]; do
    read -p "Enter the Lab Wi-Fi Name (SSID): " TARGET_SSID
    if [[ -z "$TARGET_SSID" ]]; then
        echo -e "${RED}[ERROR] SSID cannot be empty.${RESET}"
    fi
done

TARGET_PASS=""
while [[ ${#TARGET_PASS} -lt 8 ]]; do
    read -s -p "Enter the Lab Wi-Fi Password (min 8 chars): " TARGET_PASS
    echo "" # Newline for hidden input.
    
    if [[ ${#TARGET_PASS} -lt 8 ]]; then
        echo -e "${RED}[ERROR] Wi-Fi password must be at least 8 characters long (WPA2 standard).${RESET}"
        # We clear the variable so the loop repeats.
        TARGET_PASS="" 
    fi
done

STATIC_IP=""
while [[ -z "$STATIC_IP" ]]; do
    echo -e "${BOLD}[AlphaBot2]: UNIQUE Static IP is REQUIRED for this robot.${RESET}"
    read -p "[AlphaBot2]: Enter the Static IP (e.g., 192.168.X.Y): " STATIC_IP
    
    if [[ -z "$STATIC_IP" ]]; then
        echo -e "${RED}[ERROR] Input cannot be empty. Please enter a valid IP address.${RESET}"
        echo ""
    fi
done

DEFAULT_GW=$(echo "$STATIC_IP" | cut -d'.' -f1-3).1
read -p "Enter the Lab Gateway IP (Default: $DEFAULT_GW): " USER_GW_INPUT
TARGET_GATEWAY=${USER_GW_INPUT:-$DEFAULT_GW}

echo ""
echo -e "[Config] Workspace:  ${BOLD}$WORKSPACE_DIR${RESET}"
echo -e "[Config] Swap Size:  ${BOLD}$SWAP_SIZE${RESET}"
echo -e "[Config] Wi-Fi SSID: ${BOLD}$TARGET_SSID${RESET}"
echo -e "[Config] Static IP:  ${BOLD}$STATIC_IP${RESET}"
echo -e "[Config] Gateway:    ${BOLD}$TARGET_GATEWAY${RESET}"
echo ""

echo -e "[AlphaBot2]: Workspace will be created at: ${BOLD}$WORKSPACE_DIR${RESET}"
echo ""

# ----------------------------------------------------------------------
# [1] System Setup: Locale & Repositories
# ----------------------------------------------------------------------
echo -e "${GREEN}[Step 1] Configuring Locale and System Sources...${RESET}"

# Ensure Locale is UTF-8.
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Enable Universe.
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y

# Install the ros2-apt-source package that will configure ROS 2 repositories for the system.
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

echo ""

# ----------------------------------------------------------------------
# [2] Install ROS2 Humble Base
# ----------------------------------------------------------------------
echo -e "${GREEN}[Step 2] Installing ROS2 Humble Base...${RESET}"
sudo apt update
sudo apt upgrade -y
# Using ros-base to save space on RPi.
sudo apt install -y ros-humble-ros-base
echo ""

# ----------------------------------------------------------------------
# [3] Install Python Dependencies & RPi Tools
# ----------------------------------------------------------------------
echo -e "${GREEN}[Step 3] Installing Python dependencies and RPi GPIO tools...${RESET}"

sudo apt install -y python3-pip python3-colcon-common-extensions python3-rosdep2
pip3 install RPi.GPIO argcomplete

# Initialize rosdep.
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update
source /opt/ros/humble/setup.bash

echo ""

# ----------------------------------------------------------------------
# [4] Fix Permissions & GPIO
# ----------------------------------------------------------------------
echo -e "${GREEN}[Step 4] Setting up Permissions...${RESET}"

# Install common GPIO libs and add user to dialout.
sudo apt install -y rpi.gpio-common
sudo usermod -aG dialout,video $USER

echo -e "${BLUE}[INFO] You have been added to the 'dialout' group. You may need to logout/login for this to take effect.${RESET}"
echo ""

# ----------------------------------------------------------------------
# [5] Camera Configuration (System Level)
# ----------------------------------------------------------------------
echo -e "${GREEN}[Step 5] Configuring Camera (v4l2)...${RESET}"

sudo apt install -y v4l-utils

# Enable module.
if ! grep -q "bcm2835-v4l2" /etc/modules; then
    echo "bcm2835-v4l2" | sudo tee -a /etc/modules
    echo -e "${BLUE}[INFO] Added bcm2835-v4l2 to /etc/modules${RESET}"
fi

CONFIG_TXT="/boot/config.txt"
if [ -f "/boot/firmware/config.txt" ]; then
    CONFIG_TXT="/boot/firmware/config.txt"
fi

echo -e "${BLUE}[INFO] Editing $CONFIG_TXT...${RESET}"

# Set camera_auto_detect=0 (Disable libcamera auto-detection).
# auto-detection might cause instabilities.
if grep -q "camera_auto_detect" "$CONFIG_TXT"; then
    sudo sed -i 's/^camera_auto_detect=.*/camera_auto_detect=0/' "$CONFIG_TXT"
    echo "  - Updated: camera_auto_detect=0"
else
    echo "camera_auto_detect=0" | sudo tee -a "$CONFIG_TXT" > /dev/null
    echo "  - Added: camera_auto_detect=0"
fi

# Set start_x=1 (Enable Legacy Camera).
if grep -q "start_x" "$CONFIG_TXT"; then
    sudo sed -i 's/^start_x=.*/start_x=1/' "$CONFIG_TXT"
    echo "  - Updated: start_x=1"
else
    echo "start_x=1" | sudo tee -a "$CONFIG_TXT" > /dev/null
    echo "  - Added: start_x=1"
fi

# Set gpu_mem=128 (Required for Legacy Camera buffer).
if grep -q "gpu_mem" "$CONFIG_TXT"; then
    sudo sed -i 's/^gpu_mem=.*/gpu_mem=128/' "$CONFIG_TXT"
    echo "  - Updated: gpu_mem=128"
else
    echo "gpu_mem=128" | sudo tee -a "$CONFIG_TXT" > /dev/null
    echo "  - Added: gpu_mem=128"
fi

echo ""

# ----------------------------------------------------------------------
# [6] Workspace Setup & Cloning Repos
# ----------------------------------------------------------------------
echo -e "${GREEN}[Step 6] Setting up Workspace and Cloning Repos...${RESET}"

mkdir -p "$WORKSPACE_DIR/src"
cd "$WORKSPACE_DIR/src"

# Clone dependencies.
echo -e "${BLUE}[INFO] Cloning Camera Dependencies (Humble)...${RESET}"
# Check if dirs exist to avoid git errors on re-run.
if [ ! -d "ros2_v4l2_camera" ]; then git clone -b humble https://gitlab.com/boldhearts/ros2_v4l2_camera.git; fi
if [ ! -d "vision_opencv" ]; then git clone -b humble https://github.com/ros-perception/vision_opencv.git; fi
if [ ! -d "image_common" ]; then git clone -b humble https://github.com/ros-perception/image_common.git; fi
if [ ! -d "image_transport_plugins" ]; then git clone -b humble https://github.com/ros-perception/image_transport_plugins.git; fi

# Clone Main Alphabot Repo.
echo -e "${BLUE}[INFO] Cloning Alphabot2-ROS2 Repo...${RESET}"
if [ ! -d "alphabot2-ros2" ]; then
    git clone https://github.com/JPLDevMaster/alphabot2-ros2.git
else
    echo -e "${BLUE}[INFO] Alphabot2 repo already exists, pulling latest...${RESET}"
    cd alphabot2-ros2 && git pull && cd ..
fi

# Patch python-related dependencies (replace "-" with "_").
echo -e "${BLUE}[INFO] Patching setup.cfg to fix deprecated warnings...${RESET}"
find . -name "setup.cfg" -exec sed -i 's/script-dir/script_dir/g' {} +
find . -name "setup.cfg" -exec sed -i 's/install-scripts/install_scripts/g' {} +

echo ""

# ----------------------------------------------------------------------
# [7] Build Workspace
# ----------------------------------------------------------------------
echo -e "${GREEN}[Step 7] Building Workspace...${RESET}"

cd "$WORKSPACE_DIR"

# Install dependencies for the source code.
# We use 'ros-base' so we need to be careful to install missing build tools.
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# Dynamic SWAP creation for building on limited RAM devices like the Raspberry Pi. User can specify size or skip.
if [[ "$SWAP_SIZE" == "0" ]]; then
    echo -e "${BLUE}[INFO] User selected 0. Removing any existing swap...${RESET}"
    if [ -f /swapfile ]; then
        sudo swapoff /swapfile 2>/dev/null || true
        sudo rm -f /swapfile
    fi
else
    # If a swap file already exists, remove it so we can resize (in case the user chose a different size).
    if [ -f /swapfile ]; then
        echo -e "${BLUE}[INFO] Existing swap found. Removing it to apply new size ($SWAP_SIZE)...${RESET}"
        # Try to turn it off first (ignore errors if it was already off).
        sudo swapoff /swapfile 2>/dev/null || true
        sudo rm -f /swapfile
    fi

    # Create the fresh file with the requested size.
    echo -e "${BLUE}[INFO] Creating $SWAP_SIZE swap file...${RESET}"
    if sudo fallocate -l $SWAP_SIZE /swapfile; then
        sudo chmod 600 /swapfile
        sudo mkswap /swapfile
        sudo swapon /swapfile
        echo -e "${BLUE}[INFO] Swap created and enabled.${RESET}"
    else
        echo -e "${RED}[ERROR] Failed to create swap file (Disk full?).${RESET}"
    fi
fi

# Verify Swap status.
free -h

# Build.
colcon build --executor sequential --parallel-workers 1 --allow-overriding cv_bridge image_transport --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""

# ----------------------------------------------------------------------
# [8] Bashrc Setup
# ----------------------------------------------------------------------
echo -e "${GREEN}[Step 8] Updating ~/.bashrc...${RESET}"

MARKER_START="# ALPHABOT2_SETUP_START"
MARKER_END="# ALPHABOT2_SETUP_END"

# Clean old block.
if grep -qF "$MARKER_START" ~/.bashrc; then
    sed -i.bak "/$MARKER_START/,/$MARKER_END/d" ~/.bashrc
fi

# Add new block.
echo "" >> ~/.bashrc
echo "$MARKER_START" >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source $WORKSPACE_DIR/install/local_setup.bash" >> ~/.bashrc
echo "alias reset_network='sudo nmcli con delete \"Lab-Connection\" && sudo nmcli con delete \"Hotspot-Fallback\" && echo \"Profiles deleted. Reverting to defaults.\"' " >> ~/.bashrc
echo "$MARKER_END" >> ~/.bashrc

echo -e "${BLUE}[INFO] Added workspace sourcing to ~/.bashrc${RESET}"
echo ""

# ----------------------------------------------------------------------
# [9] Configure Failover Network (NetworkManager)
# ----------------------------------------------------------------------
echo -e "${GREEN}[Step 9] Configuring Network Manager (Failover Mode)...${RESET}"

# Install NetworkManager.
sudo apt install -y network-manager

# Clean old conflicting configurations.
echo -e "${BLUE}[INFO] Checking for conflicting Netplan files...${RESET}"

if [ -f /etc/netplan/50-cloud-init.yaml ]; then
    echo -e "${BLUE}[INFO] Removing conflicting file: 50-cloud-init.yaml${RESET}"
    sudo rm -f /etc/netplan/50-cloud-init.yaml
fi

if [ -f /etc/netplan/99-static-ip.yaml ]; then
    echo -e "${BLUE}[INFO] Removing conflicting file: 99-static-ip.yaml${RESET}"
    sudo rm -f /etc/netplan/99-static-ip.yaml
fi

# Tell Netplan to surrender control to NetworkManager.
NETPLAN_FILE="/etc/netplan/01-network-manager-all.yaml"
sudo bash -c "cat > $NETPLAN_FILE" <<EOF
network:
  version: 2
  renderer: NetworkManager
EOF
sudo netplan apply
echo -e "${BLUE}[INFO] Switched network backend to NetworkManager.${RESET}"

# Create Profiles using nmcli.
# Wait a moment for NM service to wake up.
sleep 5

# Clean up old connections that might conflict.
echo -e "${BLUE}[INFO] cleaning old connections...${RESET}"
sudo nmcli con delete "Lab-Connection" 2>/dev/null || true
sudo nmcli con delete "Hotspot-Fallback" 2>/dev/null || true
sudo nmcli con delete "preconfigured" 2>/dev/null || true

# Create PRIMARY (Lab) Connection.
echo -e "${BLUE}[INFO] Creating Primary Connection: $TARGET_SSID ...${RESET}"
sudo nmcli con add type wifi ifname wlan0 con-name "Lab-Connection" ssid "$TARGET_SSID"
sudo nmcli con modify "Lab-Connection" wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$TARGET_PASS"
sudo nmcli con modify "Lab-Connection" ipv4.method manual ipv4.addresses "$STATIC_IP/24" ipv4.gateway "$TARGET_GATEWAY" ipv4.dns "8.8.8.8,8.8.4.4"
# Set High Priority (100).
sudo nmcli con modify "Lab-Connection" connection.autoconnect-priority 100

# Create FALLBACK (Hotspot) Connection.
HOTSPOT_SSID="AlphaBot2-Hotspot"
HOTSPOT_IP="10.42.0.1"
echo -e "${BLUE}[INFO] Creating Fallback Hotspot: $HOTSPOT_SSID ...${RESET}"
sudo nmcli con add type wifi ifname wlan0 con-name "Hotspot-Fallback" ssid "$HOTSPOT_SSID" mode ap
sudo nmcli con modify "Hotspot-Fallback" wifi-sec.key-mgmt wpa-psk wifi-sec.psk "alphabot2"
sudo nmcli con modify "Hotspot-Fallback" ipv4.addresses "$HOTSPOT_IP/24" ipv4.method shared
# Set Low Priority (10).
sudo nmcli con modify "Hotspot-Fallback" connection.autoconnect-priority 10

# Disable the "Wait for Network" service so the robot boots fast even without Wi-Fi.
echo -e "${BLUE}[INFO] Disabling systemd-networkd-wait-online to speed up boot...${RESET}"
sudo systemctl disable systemd-networkd-wait-online.service
sudo systemctl mask systemd-networkd-wait-online.service

echo ""
echo -e "${GREEN}[SUCCESS] Network Configuration Complete!${RESET}"
echo -e "--------------------------------------------------------"
echo -e "1. ${BOLD}Priority Network:${RESET} $TARGET_SSID (Static IP: $STATIC_IP)"
echo -e "2. ${BOLD}Fallback Network:${RESET} $HOTSPOT_SSID (Password: 'alphabot2')"
echo -e "   -> If Lab Wi-Fi fails, Robot IP will be: $HOTSPOT_IP"
echo -e "--------------------------------------------------------"

echo ""

# ----------------------------------------------------------------------
# Final Summary
# ----------------------------------------------------------------------
echo -e "${BLUE}----------------------------------------------------------------------${RESET}"
echo -e "${BOLD}[AlphaBot2]: Installation Complete!${RESET}"
echo -e "${BLUE}----------------------------------------------------------------------${RESET}"
echo ""
echo -e "1. Please ${BOLD}reboot${RESET} your Raspberry Pi to apply camera and permission changes."
echo -e "2. After reboot, verify the camera is working with: ${BOLD}v4l2-ctl --list-devices${RESET}"
echo -e "3. To launch the robot nodes, run:"
echo -e "   ${GREEN}ros2 launch alphabot2 alphabot2_launch.py${RESET}"
echo -e "4. Keep in mind that the static IP is only for ${BOLD}Wi-Fi${RESET}! Ethernet will still be set to DHCP (Dynamic)."
echo ""
