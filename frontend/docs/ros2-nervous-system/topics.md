---
title: ROS 2 Topics - Asynchronous Communication in the Robotic Nervous System
---

# ROS 2 Topics - Asynchronous Communication in the Robotic Nervous System

## What are ROS 2 Topics?

A **Topic** is a named bus over which nodes exchange messages asynchronously, forming a many-to-many communication pattern. Topics are the primary mechanism for asynchronous communication in ROS 2 and represent the "synapses" of the robotic nervous system, enabling nodes to share information without direct coordination.

## Key Characteristics of Topics

### Asynchronous Communication
- Publishers send messages without waiting for responses
- Subscribers receive messages when they are available
- No direct connection between publishers and subscribers

### Many-to-Many Pattern
- Multiple publishers can send to the same topic
- Multiple subscribers can receive from the same topic
- Decouples publishers from subscribers in time and space

### Message Type
- Each topic has a specific message type (e.g., `std_msgs/String`, `sensor_msgs/LaserScan`)
- All publishers and subscribers on a topic must use the same message type
- Message types are defined using `.msg` files

## Creating Publishers and Subscribers

### Publisher Example
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1
```

### Subscriber Example
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

## Quality of Service (QoS) Profiles

QoS profiles control how messages are delivered between publishers and subscribers:

### Reliability
- **Reliable**: All messages are guaranteed to be delivered
- **Best Effort**: Messages may be lost, but higher throughput

### Durability
- **Transient Local**: Late-joining subscribers receive the last message
- **Volatile**: No messages are stored for late joiners

### History
- **Keep Last**: Store the most recent N messages
- **Keep All**: Store all messages (use with caution)

### Example with QoS
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Create a custom QoS profile
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

self.publisher_ = self.create_publisher(String, 'chatter', qos_profile)
```

## Topic Commands

### Listing Topics
```bash
ros2 topic list
```

### Getting Topic Information
```bash
ros2 topic info /chatter
```

### Echoing Topic Data
```bash
ros2 topic echo /chatter std_msgs/msg/String
```

### Publishing to a Topic (Command Line)
```bash
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello World'"
```

## Message Types

### Common Built-in Types
- `std_msgs`: Basic data types (String, Int32, Float64, etc.)
- `sensor_msgs`: Sensor data (LaserScan, Image, PointCloud2, etc.)
- `geometry_msgs`: Geometric primitives (Point, Pose, Twist, etc.)
- `nav_msgs`: Navigation messages (Odometry, Path, OccupancyGrid, etc.)

### Creating Custom Message Types
Custom message types are defined in `.msg` files in the `msg/` directory of a package:

```
# Custom message example: MyMessage.msg
string name
int32 id
float64[] values
geometry_msgs/Point position
```

## Advanced Topic Features

### Latching
Latching allows publishers to send the last message to late-joining subscribers:

```python
# This is now handled by Transient Local durability QoS
```

### Publisher/Subscriber Count
Check how many publishers or subscribers are connected:

```python
# Get number of publishers for a topic
publisher_count = self.publisher_.get_subscription_count()

# Get number of subscribers for a topic (from subscriber side)
# This is not directly available, but you can infer from message rates
```

### Topic Remapping
Remap topic names at runtime:

```python
# In Python launch file
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='talker',
            name='talker',
            remappings=[('chatter', 'custom_chatter')]
        )
    ])
```

## Topic Performance Considerations

### Bandwidth
- High-frequency topics can consume significant bandwidth
- Use appropriate message types (e.g., compressed images)
- Consider throttling for high-bandwidth topics

### Latency
- Choose appropriate QoS settings for your application
- Monitor message delay with tools like `ros2 topic hz`

### Memory Usage
- QoS history depth affects memory consumption
- Be careful with "Keep All" history policy

## The Role of Topics in the Robotic Nervous System

Topics function as the primary communication mechanism in the robotic nervous system:

- **Sensors**: Publish sensor data (camera images, LIDAR scans, IMU readings)
- **Controllers**: Publish commands to actuators
- **Perception**: Publish detected objects, maps, or other processed data
- **Behavior**: Publish status, goals, or other coordination information

This asynchronous communication pattern allows for flexible, decoupled robot software architectures where nodes can be added, removed, or modified without affecting other parts of the system.

## Common Patterns

### Sensor Data Distribution
Multiple perception nodes can subscribe to the same sensor data without affecting the sensor node.

### Command Distribution
A single controller can publish commands to multiple actuator nodes simultaneously.

### Status Broadcasting
Nodes can broadcast their status to multiple monitoring or logging nodes.

## Next Steps

Now that you understand ROS 2 Topics, continue to learn about:
- **Services**: Synchronous request-response communication
- **Actions**: Goal-oriented communication for long-running tasks
- **Architecture**: How nodes and topics work together in the ROS 2 graph