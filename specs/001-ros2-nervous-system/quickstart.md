# Quickstart Guide: Module 1 - The Robotic Nervous System (ROS 2)

## Overview
This guide will help you get started with the ROS 2 nervous system examples. You'll learn about Nodes, Topics, Services, and URDF through practical examples.

## Prerequisites

### System Requirements
- Ubuntu 22.04 (for ROS 2 Humble Hawksbill)
- Python 3.10 or higher
- At least 4GB RAM (8GB recommended)
- NVIDIA Jetson Orin Nano (for hardware deployment) or development machine for simulation

### ROS 2 Installation
1. Set up your sources:
   ```bash
   sudo apt update && sudo apt install curl gnupg lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros-rolling.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

2. Install ROS 2 Humble:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   sudo apt install python3-rosdep2
   ```

3. Source the ROS 2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

4. (Optional) Add to your bashrc to auto-source:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```

## Getting Started with Examples

### 1. Clone the Repository
```bash
git clone https://github.com/your-organization/physical-ai-book.git
cd physical-ai-book
```

### 2. Navigate to ROS 2 Examples
```bash
cd src/ros2_examples
```

### 3. Hello World Publisher/Subscriber Example

#### Terminal 1 - Start the Publisher:
```bash
cd publisher_subscriber
source /opt/ros/humble/setup.bash
python3 publisher_node.py
```

#### Terminal 2 - Start the Subscriber:
```bash
cd publisher_subscriber
source /opt/ros/humble/setup.bash
python3 subscriber_node.py
```

You should see the subscriber receiving "Hello World" messages from the publisher.

### 4. Service Example

#### Terminal 1 - Start the Service Server:
```bash
cd services
source /opt/ros/humble/setup.bash
python3 service_server.py
```

#### Terminal 2 - Call the Service:
```bash
cd services
source /opt/ros/humble/setup.bash
python3 service_client.py
```

You should see the client request the service and receive the computed sum.

## Understanding the Concepts

### Nodes
- Each Python file represents a ROS 2 Node
- Nodes are the basic execution units in ROS 2
- They can publish messages, subscribe to topics, and provide services

### Topics
- Named buses for asynchronous communication
- Publisher nodes send messages to topics
- Subscriber nodes receive messages from topics
- Communication is many-to-many (one publisher, multiple subscribers)

### Services
- Synchronous request/response communication
- Service server provides a service
- Service client calls the service and waits for a response
- Communication is one-to-one

### URDF
- Unified Robot Description Format
- XML format to describe robot structure
- Defines links (physical parts) and joints (connections)
- Used for simulation and visualization

## Running on Jetson Orin Nano

When deploying to the Jetson Orin Nano:
1. Ensure ROS 2 Humble is installed on the device
2. Copy the example files to the Jetson
3. Source the ROS 2 environment: `source /opt/ros/humble/setup.bash`
4. Run the examples as described above

## Simulation vs Real Hardware

### Simulation (Isaac Sim/Gazebo)
- Use for development and testing
- Faster iteration
- Safe environment for testing

### Real Hardware (Jetson Orin Nano)
- Use for final validation
- Real-world performance characteristics
- Actual robot interaction

## Troubleshooting

### Common Issues:
- **"Command 'ros2' not found"**: Make sure you've sourced the ROS 2 environment
- **Permission errors**: Check your ROS 2 installation and environment setup
- **Nodes can't communicate**: Verify both nodes are on the same ROS domain

### Useful Commands:
- `ros2 node list` - List all active nodes
- `ros2 topic list` - List all active topics
- `ros2 service list` - List all available services
- `ros2 run <package> <executable>` - Run a ROS 2 node

## Next Steps
1. Experiment with modifying the example code
2. Create your own custom message types
3. Add more complex robot models with URDF
4. Explore advanced ROS 2 features like Actions and Parameters

## Complete Example Code

### Publisher Node (`publisher_member_function.py`)

Create a file named `publisher_member_function.py` with the following content:

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Subscriber Node (`subscriber_member_function.py`)

Create a file named `subscriber_member_function.py` with the following content:

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Running the Nodes

After creating these files:

1. Create a setup.py file with entry points:
```python
    entry_points={
        'console_scripts': [
            'talker = my_ros2_package.publisher_member_function:main',
            'listener = my_ros2_package.subscriber_member_function:main',
        ],
    },
```

2. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select my_ros2_package
```

3. Run the nodes in separate terminals:
```bash
# Terminal 1 (Publisher):
ros2 run my_ros2_package talker

# Terminal 2 (Subscriber):
ros2 run my_ros2_package listener
```

You should see the publisher sending "Hello World" messages and the subscriber receiving them.
