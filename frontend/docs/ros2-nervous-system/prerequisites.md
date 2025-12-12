---
title: Prerequisites for ROS 2 Development on Jetson Orin Nano
---

# Prerequisites for ROS 2 Development on Jetson Orin Nano

Before diving into ROS 2 development, you need to set up your environment properly. This guide covers the prerequisites for developing ROS 2 applications, with special attention to the Jetson Orin Nano platform.

## System Requirements

### For Development Environment
- **Operating System**: Ubuntu 22.04 LTS (recommended) or other Linux distributions supporting ROS 2 Humble Hawksbill
- **RAM**: At least 4GB (8GB recommended)
- **Storage**: At least 20GB of free space
- **Processor**: Multi-core processor (x86_64 or ARM64)

### For Jetson Orin Nano
- **JetPack SDK**: Version 5.1 or higher (includes Ubuntu 20.04 LTS)
- **RAM**: 8GB or 16GB LPDDR5
- **Storage**: 32GB or 64GB eMMC or NVMe SSD
- **Power Supply**: 19V/65W AC adapter for stable operation

## ROS 2 Installation

### ROS 2 Humble Hawksbill (Recommended)
ROS 2 Humble Hawksbill is an LTS (Long Term Support) version that provides stability and long-term support. It's the recommended version for production and learning purposes.

#### Installation on Ubuntu 22.04

1. **Set up locale**:
   ```bash
   locale-gen en_US.UTF-8
   export LANG=en_US.UTF-8
   ```

2. **Set up sources**:
   ```bash
   sudo apt update && sudo apt install curl gnupg lsb-release
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros-keyring.gpg | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

3. **Install ROS 2**:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   sudo apt install python3-rosdep2
   ```

4. **Initialize rosdep**:
   ```bash
   sudo rosdep init
   rosdep update
   ```

5. **Environment setup**:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

### Python Dependencies
```bash
pip3 install ros-humble-rclpy
pip3 install ros-humble-std-msgs
pip3 install ros-humble-example-interfaces
```

## Jetson Orin Nano Specific Setup

### JetPack Installation
1. Download the latest JetPack SDK from NVIDIA Developer website
2. Follow NVIDIA's official installation guide for flashing Jetson Orin Nano
3. Ensure you have the latest JetPack version for optimal ROS 2 performance

### Jetson ROS 2 Installation
On the Jetson Orin Nano, you can install ROS 2 using the same commands as above, or use the pre-built containers from NVIDIA that are optimized for Jetson platforms.

## Development Tools

### Required Tools
- **Git**: For version control
- **Python 3.8+**: For ROS 2 Python packages
- **Colcon**: ROS 2 build system
- **Vim/VSCode**: Code editor with ROS 2 extensions

### Recommended Tools
- **RViz2**: 3D visualization tool for ROS 2
- **rqt**: GUI tools for introspecting ROS 2 topics and services
- **Gazebo/Humble**: Simulation environment

## Network Configuration (Important for Distributed Systems)

### ROS 2 Domain ID
ROS 2 uses domain IDs to separate different ROS 2 networks. Set your domain ID:
```bash
export ROS_DOMAIN_ID=42  # Use any number 0-101
```

### Network Setup for Multi-machine Communication
If you plan to run nodes across multiple machines:
```bash
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=42
```

## Testing Your Installation

Verify your ROS 2 installation:
```bash
ros2 topic list
ros2 node list
ros2 service list
```

## Troubleshooting Common Issues

### "Command 'ros2' not found"
- Make sure you've sourced the ROS 2 environment: `source /opt/ros/humble/setup.bash`
- Add the source command to your `.bashrc` file

### Permission Issues
- Check that your user is in the correct groups: `sudo usermod -a -G dialout $USER`
- Log out and log back in for changes to take effect

### Network Communication Issues
- Verify that ROS_DOMAIN_ID is the same across all machines
- Check firewall settings that might block ROS 2 communication

## Next Steps

Once you have completed the prerequisites setup, you can proceed to:

1. Learn about ROS 2 Nodes
2. Understand ROS 2 Topics and messaging
3. Explore ROS 2 Services
4. Work with URDF for robot modeling

Continue to the next section to learn about ROS 2 Nodes, the fundamental building blocks of the robotic nervous system.