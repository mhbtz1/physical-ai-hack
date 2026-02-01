#!/bin/bash
set -e

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
echo_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
echo_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# ============================================================================
# SYSTEM PREREQUISITES
# ============================================================================
echo_info "Checking Ubuntu version..."
UBUNTU_CODENAME=$(lsb_release -sc)
echo_info "Ubuntu version: $UBUNTU_CODENAME"

if [ "$UBUNTU_CODENAME" != "jammy" ]; then
    echo_warn "This script is designed for Ubuntu 22.04 (Jammy)"
    echo_warn "You have: $UBUNTU_CODENAME"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo_info "Updating system packages..."
sudo apt update
sudo apt upgrade -y

echo_info "Installing build essentials..."
sudo apt install -y \
    build-essential \
    software-properties-common \
    curl \
    wget \
    gnupg \
    gnupg2 \
    lsb-release \
    ca-certificates \
    apt-transport-https \
    git \
    cmake

# Setup locale
echo_info "Configuring UTF-8 locale..."
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ============================================================================
# ROS 2 HUMBLE INSTALLATION
# ============================================================================
echo_info "Setting up ROS 2 repository..."

# Add universe repository
sudo add-apt-repository universe -y
sudo apt update

# Add ROS 2 GPG key via keyserver (avoids HTTPS issues)
echo_info "Adding ROS 2 GPG key..."
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 \
    --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Add ROS 2 repository
echo_info "Adding ROS 2 apt repository..."
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

# Install ROS 2
sudo apt update
echo_info "Installing ROS 2 Humble Desktop (this will take several minutes)..."
sudo apt install -y ros-humble-desktop

# Install development tools
echo_info "Installing ROS 2 development tools..."
sudo apt install -y \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# Initialize rosdep
echo_info "Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# ============================================================================
# C++ ROBOTICS LIBRARIES
# ============================================================================
echo_info "Installing C++ development tools..."
sudo apt install -y \
    gdb \
    clang \
    clang-format \
    clang-tidy \
    valgrind

echo_info "Installing mathematics and transformation libraries..."
sudo apt install -y \
    libeigen3-dev \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-eigen \
    ros-humble-tf2-geometry-msgs \
    ros-humble-eigen3-cmake-module

echo_info "Installing computer vision libraries..."
sudo apt install -y \
    libopencv-dev \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    ros-humble-image-transport \
    ros-humble-image-proc \
    ros-humble-camera-calibration-parsers

echo_info "Installing point cloud processing libraries..."
sudo apt install -y \
    libpcl-dev \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-perception-pcl

echo_info "Installing motion planning libraries..."
sudo apt install -y \
    ros-humble-moveit \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-visual-tools \
    libompl-dev

echo_info "Installing kinematics and dynamics libraries..."
sudo apt install -y \
    liborocos-kdl-dev \
    ros-humble-kdl-parser \
    ros-humble-pinocchio

echo_info "Installing control system libraries..."
sudo apt install -y \
    ros-humble-control-toolbox \
    ros-humble-realtime-tools \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager

echo_info "Installing utility libraries..."
sudo apt install -y \
    libboost-all-dev \
    libfmt-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    libceres-dev \
    ros-humble-message-filters

# ============================================================================
# PYTHON DEPENDENCIES
# ============================================================================
echo_info "Installing Python development tools..."
sudo apt install -y \
    python3-pip \
    python3-dev \
    python3-venv \
    python3-setuptools \
    python3-wheel

echo_info "Installing Python scientific computing stack..."
pip3 install --break-system-packages \
    numpy \
    scipy \
    scikit-learn \
    matplotlib \
    seaborn \
    pandas

echo_info "Installing PyTorch (CPU version)..."
pip3 install --break-system-packages \
    torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

# For GPU instances, uncomment this instead:
# echo_info "Installing PyTorch (GPU version)..."
# pip3 install --break-system-packages \
#     torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

echo_info "Installing reinforcement learning and imitation learning libraries..."
pip3 install --break-system-packages \
    stable-baselines3 \
    gymnasium \
    gym

echo_info "Installing computer vision libraries..."
pip3 install --break-system-packages \
    opencv-python \
    opencv-contrib-python \
    pillow \
    imageio \
    imageio-ffmpeg

echo_info "Installing data handling libraries..."
pip3 install --break-system-packages \
    h5py \
    pytables \
    msgpack \
    protobuf \
    pyyaml

echo_info "Installing transformer and modern ML libraries..."
pip3 install --break-system-packages \
    transformers \
    timm \
    einops

echo_info "Installing experiment tracking tools..."
pip3 install --break-system-packages \
    wandb \
    tensorboard

echo_info "Installing ROS Python utilities..."
pip3 install --break-system-packages \
    rospkg \
    catkin-pkg

echo_info "Installing visualization tools..."
pip3 install --break-system-packages \
    rerun-sdk \
    jupyter \
    ipython

# ============================================================================
# SIMULATION AND VISUALIZATION
# ============================================================================
echo_info "Installing Gazebo simulation..."
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control

echo_info "Installing visualization tools..."
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rqt-robot-plugins

# ============================================================================
# SENSOR DRIVERS
# ============================================================================
echo_info "Installing sensor packages..."
sudo apt install -y \
    ros-humble-usb-cam \
    ros-humble-realsense2-camera \
    ros-humble-image-pipeline \
    ros-humble-laser-geometry \
    ros-humble-laser-filters

# ============================================================================
# DATA RECORDING
# ============================================================================
echo_info "Installing rosbag and data recording tools..."
sudo apt install -y \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-default-plugins \
    ros-humble-rosbag2-storage-sqlite3 \
    ros-humble-rosbag2-compression-zstd \
    ros-humble-ros2bag \
    ros-humble-rqt-bag

# ============================================================================
# TURTLEBOT3 FOR TESTING
# ============================================================================
echo_info "Installing TurtleBot3 packages..."
sudo apt install -y \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3-simulations \
    ros-humble-turtlebot3-gazebo \
    ros-humble-dynamixel-sdk \
    ros-humble-turtlebot3-cartographer \
    ros-humble-turtlebot3-navigation2

# ============================================================================
# WORKSPACE SETUP
# ============================================================================
echo_info "Creating ROS 2 workspace..."
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# ============================================================================
# ENVIRONMENT CONFIGURATION
# ============================================================================
echo_info "Configuring shell environment..."

# ROS 2 setup
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS 2 Humble" >> ~/.bashrc
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

# Workspace setup
if ! grep -q "source ~/ros2_ws/install/setup.bash" ~/.bashrc; then
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
fi

# TurtleBot3 configuration
if ! grep -q "export TURTLEBOT3_MODEL" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# TurtleBot3" >> ~/.bashrc
    echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
fi

# Gazebo model path
if ! grep -q "GAZEBO_MODEL_PATH.*turtlebot3" ~/.bashrc; then
    echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> ~/.bashrc
fi

# ROS Domain ID
if ! grep -q "ROS_DOMAIN_ID" ~/.bashrc; then
    echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
fi

# ============================================================================
# VERIFICATION
# ============================================================================
echo_info "Verifying installation..."
source /opt/ros/humble/setup.bash

# Check ROS 2
if command -v ros2 &> /dev/null; then
    echo_info "✓ ROS 2 installed: $(ros2 --version)"
else
    echo_error "✗ ROS 2 installation failed"
    exit 1
fi

# Check Python packages
if python3 -c "import torch, numpy, cv2" 2>/dev/null; then
    echo_info "✓ Python ML stack installed"
else
    echo_warn "⚠ Some Python packages may be missing"
fi

# Check TurtleBot3
if ros2 pkg list 2>/dev/null | grep -q turtlebot3; then
    echo_info "✓ TurtleBot3 packages installed"
else
    echo_warn "⚠ TurtleBot3 packages may be incomplete"
fi

# Check Gazebo
if command -v gazebo &> /dev/null; then
    echo_info "✓ Gazebo installed"
else
    echo_warn "⚠ Gazebo may not be installed correctly"
fi

# ============================================================================
# COMPLETION
# ============================================================================
echo ""
echo "=========================================="
echo_info "Installation Complete!"
echo "=========================================="
echo ""
echo "Installed components:"
echo "  • ROS 2 Humble Desktop"
echo "  • C++ robotics libraries (Eigen, OpenCV, PCL, MoveIt, KDL)"
echo "  • Python ML stack (PyTorch, NumPy, scikit-learn)"
echo "  • Imitation learning tools (stable-baselines3, gymnasium)"
echo "  • Simulation (Gazebo, TurtleBot3)"
echo "  • Visualization (RViz2, Rerun)"
echo "  • Data recording (rosbag2)"
echo ""
echo "Next steps:"
echo "  1. Reload environment: source ~/.bashrc"
echo "  2. Verify: ros2 --version"
echo "  3. Test simulation: ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
echo ""
echo "Workspace: ~/ros2_ws"
echo "=========================================="

# Mark completion
touch ~/.ros2_setup_complete
date >> ~/.ros2_setup_complete