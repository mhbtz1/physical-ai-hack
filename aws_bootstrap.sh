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
UBUNTU_VERSION=$(lsb_release -rs)
echo_info "Ubuntu version: $UBUNTU_VERSION ($UBUNTU_CODENAME)"

# Determine ROS version based on Ubuntu version
if [ "$UBUNTU_CODENAME" = "noble" ] || [ "$UBUNTU_VERSION" = "24.04" ]; then
    ROS_DISTRO="jazzy"
    echo_info "Using ROS 2 Jazzy for Ubuntu 24.04"
elif [ "$UBUNTU_CODENAME" = "jammy" ] || [ "$UBUNTU_VERSION" = "22.04" ]; then
    ROS_DISTRO="humble"
    echo_info "Using ROS 2 Humble for Ubuntu 22.04"
elif [ "$UBUNTU_CODENAME" = "focal" ] || [ "$UBUNTU_VERSION" = "20.04" ]; then
    ROS_DISTRO="foxy"
    echo_info "Using ROS 2 Foxy for Ubuntu 20.04"
else
    echo_error "Unsupported Ubuntu version: $UBUNTU_VERSION ($UBUNTU_CODENAME)"
    echo_error "This script supports Ubuntu 20.04, 22.04, and 24.04"
    exit 1
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
# ROS 2 INSTALLATION
# ============================================================================
echo_info "Setting up ROS 2 $ROS_DISTRO repository..."

# Add universe repository
sudo add-apt-repository universe -y
sudo apt update

# For Ubuntu 24.04, use modern GPG key management
if [ "$UBUNTU_CODENAME" = "noble" ]; then
    echo_info "Using modern GPG key management for Ubuntu 24.04..."
    sudo mkdir -p /etc/apt/keyrings
    sudo wget -qO /etc/apt/keyrings/ros-archive-keyring.gpg \
        https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc || \
        curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
        sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg
    
    # Add ROS 2 repository with signed-by
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" | \
        sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
else
    # Use older apt-key method for Ubuntu 22.04 and earlier
    echo_info "Using apt-key for Ubuntu $UBUNTU_VERSION..."
    sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 \
        --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    
    # Add ROS 2 repository
    sudo sh -c "echo 'deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main' > /etc/apt/sources.list.d/ros2.list"
fi

# Install ROS 2
sudo apt update
echo_info "Installing ROS 2 $ROS_DISTRO Desktop (this will take several minutes)..."
sudo apt install -y ros-${ROS_DISTRO}-desktop

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
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-eigen \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-eigen3-cmake-module

echo_info "Installing computer vision libraries..."
sudo apt install -y \
    libopencv-dev \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-vision-opencv \
    ros-${ROS_DISTRO}-image-transport

# Image processing packages - check if available for this distro
if apt-cache show ros-${ROS_DISTRO}-image-proc &> /dev/null; then
    sudo apt install -y ros-${ROS_DISTRO}-image-proc
fi

if apt-cache show ros-${ROS_DISTRO}-camera-calibration-parsers &> /dev/null; then
    sudo apt install -y ros-${ROS_DISTRO}-camera-calibration-parsers
fi

echo_info "Installing point cloud processing libraries..."
sudo apt install -y libpcl-dev

# PCL ROS packages - check availability
if apt-cache show ros-${ROS_DISTRO}-pcl-ros &> /dev/null; then
    sudo apt install -y \
        ros-${ROS_DISTRO}-pcl-ros \
        ros-${ROS_DISTRO}-pcl-conversions
fi

if apt-cache show ros-${ROS_DISTRO}-perception-pcl &> /dev/null; then
    sudo apt install -y ros-${ROS_DISTRO}-perception-pcl
fi

echo_info "Installing motion planning libraries..."
# MoveIt availability varies by distro
if apt-cache show ros-${ROS_DISTRO}-moveit &> /dev/null; then
    sudo apt install -y \
        ros-${ROS_DISTRO}-moveit \
        ros-${ROS_DISTRO}-moveit-ros-planning-interface
    
    if apt-cache show ros-${ROS_DISTRO}-moveit-visual-tools &> /dev/null; then
        sudo apt install -y ros-${ROS_DISTRO}-moveit-visual-tools
    fi
else
    echo_warn "MoveIt not available for ROS 2 $ROS_DISTRO yet"
fi

sudo apt install -y libompl-dev

echo_info "Installing kinematics and dynamics libraries..."
sudo apt install -y liborocos-kdl-dev

if apt-cache show ros-${ROS_DISTRO}-kdl-parser &> /dev/null; then
    sudo apt install -y ros-${ROS_DISTRO}-kdl-parser
fi

if apt-cache show ros-${ROS_DISTRO}-pinocchio &> /dev/null; then
    sudo apt install -y ros-${ROS_DISTRO}-pinocchio
fi

echo_info "Installing control system libraries..."
if apt-cache show ros-${ROS_DISTRO}-control-toolbox &> /dev/null; then
    sudo apt install -y ros-${ROS_DISTRO}-control-toolbox
fi

if apt-cache show ros-${ROS_DISTRO}-realtime-tools &> /dev/null; then
    sudo apt install -y ros-${ROS_DISTRO}-realtime-tools
fi

sudo apt install -y \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-controller-manager

echo_info "Installing utility libraries..."
sudo apt install -y \
    libboost-all-dev \
    libfmt-dev \
    libspdlog-dev \
    libyaml-cpp-dev

# Ceres solver
if apt-cache show libceres-dev &> /dev/null; then
    sudo apt install -y libceres-dev
fi

if apt-cache show ros-${ROS_DISTRO}-message-filters &> /dev/null; then
    sudo apt install -y ros-${ROS_DISTRO}-message-filters
fi

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
#     torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

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
if apt-cache show ros-${ROS_DISTRO}-gazebo-ros-pkgs &> /dev/null; then
    sudo apt install -y ros-${ROS_DISTRO}-gazebo-ros-pkgs
fi

if apt-cache show ros-${ROS_DISTRO}-gazebo-ros2-control &> /dev/null; then
    sudo apt install -y ros-${ROS_DISTRO}-gazebo-ros2-control
fi

echo_info "Installing visualization tools..."
sudo apt install -y ros-${ROS_DISTRO}-rviz2

if apt-cache show ros-${ROS_DISTRO}-rqt &> /dev/null; then
    sudo apt install -y \
        ros-${ROS_DISTRO}-rqt \
        ros-${ROS_DISTRO}-rqt-common-plugins
fi

if apt-cache show ros-${ROS_DISTRO}-rqt-robot-plugins &> /dev/null; then
    sudo apt install -y ros-${ROS_DISTRO}-rqt-robot-plugins
fi

# ============================================================================
# SENSOR DRIVERS
# ============================================================================
echo_info "Installing sensor packages..."
if apt-cache show ros-${ROS_DISTRO}-usb-cam &> /dev/null; then
    sudo apt install -y ros-${ROS_DISTRO}-usb-cam
fi

if apt-cache show ros-${ROS_DISTRO}-realsense2-camera &> /dev/null; then
    sudo apt install -y ros-${ROS_DISTRO}-realsense2-camera
fi

if apt-cache show ros-${ROS_DISTRO}-image-pipeline &> /dev/null; then
    sudo apt install -y ros-${ROS_DISTRO}-image-pipeline
fi

if apt-cache show ros-${ROS_DISTRO}-laser-geometry &> /dev/null; then
    sudo apt install -y ros-${ROS_DISTRO}-laser-geometry
fi

if apt-cache show ros-${ROS_DISTRO}-laser-filters &> /dev/null; then
    sudo apt install -y ros-${ROS_DISTRO}-laser-filters
fi

# ============================================================================
# DATA RECORDING
# ============================================================================
echo_info "Installing rosbag and data recording tools..."
sudo apt install -y \
    ros-${ROS_DISTRO}-rosbag2 \
    ros-${ROS_DISTRO}-rosbag2-storage-default-plugins \
    ros-${ROS_DISTRO}-ros2bag

if apt-cache show ros-${ROS_DISTRO}-rosbag2-storage-sqlite3 &> /dev/null; then
    sudo apt install -y ros-${ROS_DISTRO}-rosbag2-storage-sqlite3
fi

if apt-cache show ros-${ROS_DISTRO}-rosbag2-compression-zstd &> /dev/null; then
    sudo apt install -y ros-${ROS_DISTRO}-rosbag2-compression-zstd
fi

if apt-cache show ros-${ROS_DISTRO}-rqt-bag &> /dev/null; then
    sudo apt install -y ros-${ROS_DISTRO}-rqt-bag
fi

# ============================================================================
# TURTLEBOT3 FOR TESTING
# ============================================================================
echo_info "Installing TurtleBot3 packages..."
if apt-cache show ros-${ROS_DISTRO}-turtlebot3 &> /dev/null; then
    sudo apt install -y \
        ros-${ROS_DISTRO}-turtlebot3 \
        ros-${ROS_DISTRO}-turtlebot3-msgs
    
    if apt-cache show ros-${ROS_DISTRO}-turtlebot3-simulations &> /dev/null; then
        sudo apt install -y ros-${ROS_DISTRO}-turtlebot3-simulations
    fi
    
    if apt-cache show ros-${ROS_DISTRO}-turtlebot3-gazebo &> /dev/null; then
        sudo apt install -y ros-${ROS_DISTRO}-turtlebot3-gazebo
    fi
    
    if apt-cache show ros-${ROS_DISTRO}-dynamixel-sdk &> /dev/null; then
        sudo apt install -y ros-${ROS_DISTRO}-dynamixel-sdk
    fi
    
    if apt-cache show ros-${ROS_DISTRO}-turtlebot3-cartographer &> /dev/null; then
        sudo apt install -y ros-${ROS_DISTRO}-turtlebot3-cartographer
    fi
    
    if apt-cache show ros-${ROS_DISTRO}-turtlebot3-navigation2 &> /dev/null; then
        sudo apt install -y ros-${ROS_DISTRO}-turtlebot3-navigation2
    fi
else
    echo_warn "TurtleBot3 packages not yet available for ROS 2 $ROS_DISTRO"
    echo_warn "You may need to build from source or wait for official packages"
fi

# ============================================================================
# WORKSPACE SETUP
# ============================================================================
echo_info "Creating ROS 2 workspace..."
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install

# ============================================================================
# ENVIRONMENT CONFIGURATION
# ============================================================================
echo_info "Configuring shell environment..."

# ROS 2 setup
if ! grep -q "source /opt/ros/${ROS_DISTRO}/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS 2 ${ROS_DISTRO}" >> ~/.bashrc
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
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
    echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models" >> ~/.bashrc
fi

# ROS Domain ID
if ! grep -q "ROS_DOMAIN_ID" ~/.bashrc; then
    echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
fi

# ============================================================================
# VERIFICATION
# ============================================================================
echo_info "Verifying installation..."
source /opt/ros/${ROS_DISTRO}/setup.bash

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
    TORCH_VERSION=$(python3 -c "import torch; print(torch.__version__)")
    echo_info "  PyTorch version: $TORCH_VERSION"
else
    echo_warn "⚠ Some Python packages may be missing"
fi

# Check TurtleBot3
if ros2 pkg list 2>/dev/null | grep -q turtlebot3; then
    echo_info "✓ TurtleBot3 packages installed"
else
    echo_warn "⚠ TurtleBot3 packages not available for this ROS version"
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
echo "ROS Distribution: ROS 2 $ROS_DISTRO"
echo "Ubuntu Version: $UBUNTU_VERSION ($UBUNTU_CODENAME)"
echo ""
echo "Installed components:"
echo "  • ROS 2 ${ROS_DISTRO} Desktop"
echo "  • C++ robotics libraries (Eigen, OpenCV, PCL, KDL)"
if apt-cache show ros-${ROS_DISTRO}-moveit &> /dev/null; then
    echo "  • MoveIt motion planning"
fi
echo "  • Python ML stack (PyTorch, NumPy, scikit-learn)"
echo "  • Imitation learning tools (stable-baselines3, gymnasium)"
echo "  • Simulation (Gazebo)"
echo "  • Visualization (RViz2, Rerun)"
echo "  • Data recording (rosbag2)"
if ros2 pkg list 2>/dev/null | grep -q turtlebot3; then
    echo "  • TurtleBot3 simulation"
fi
echo ""
echo "Next steps:"
echo "  1. Reload environment: source ~/.bashrc"
echo "  2. Verify: ros2 --version"
if ros2 pkg list 2>/dev/null | grep -q turtlebot3; then
    echo "  3. Test simulation: ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
fi
echo ""
echo "Workspace: ~/ros2_ws"
echo "=========================================="

# Mark completion
touch ~/.ros2_setup_complete
echo "ROS 2 $ROS_DISTRO on Ubuntu $UBUNTU_VERSION" > ~/.ros2_setup_complete
date >> ~/.ros2_setup_complete
