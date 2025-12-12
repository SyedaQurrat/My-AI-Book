---
title: ROS 2 Nodes - The Building Blocks of the Robotic Nervous System
---

# ROS 2 Nodes - The Building Blocks of the Robotic Nervous System

## What is a ROS 2 Node?

A **Node** is the fundamental building block of a ROS 2 system. It's an executable process that performs computation and is often a single-purpose component in the robotic system. Nodes are the "neurons" of the robotic nervous system, processing information and coordinating with other nodes to achieve complex robotic behaviors.

## Key Characteristics of Nodes

### Identity and Organization
- **Name**: Each node has a unique identifier within the ROS 2 graph
- **Namespace**: Optional namespace for organizing nodes hierarchically
- **Parameters**: Configuration values that can be set at runtime

### Communication Capabilities
- **Publishers**: Create publishers to send messages to topics
- **Subscribers**: Create subscribers to receive messages from topics
- **Services**: Provide services that other nodes can call
- **Clients**: Create clients to call services provided by other nodes

## Creating a Node in Python

Here's the basic structure of a ROS 2 node in Python using the `rclpy` client library:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Node initialization code goes here
        self.get_logger().info('MyNode has been started')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Node Lifecycle

### Initialization
1. **Node Creation**: Initialize the node with a unique name
2. **Component Setup**: Create publishers, subscribers, services, and clients
3. **Parameter Declaration**: Define and read parameters

### Execution
1. **Spinning**: The node runs and processes callbacks
2. **Communication**: Send/receive messages, provide services
3. **Computation**: Perform the primary task of the node

### Shutdown
1. **Cleanup**: Destroy publishers, subscribers, and services
2. **Resource Release**: Free allocated resources
3. **Termination**: Node process ends gracefully

## Best Practices for Node Design

### Single Responsibility Principle
Each node should have a single, well-defined purpose. This makes the system more modular and easier to debug.

### Proper Resource Management
Always clean up resources when the node shuts down:
- Destroy publishers and subscribers
- Close file handles
- Release memory

### Error Handling
Implement proper error handling to ensure the node can recover from unexpected situations:
- Use try-catch blocks where appropriate
- Log errors for debugging
- Implement fallback behaviors

### Parameter Configuration
Use ROS 2 parameters to make nodes configurable:
- Declare parameters with default values
- Allow runtime configuration
- Validate parameter values

## Node Communication Patterns

### Publisher-Subscriber (Topics)
Asynchronous communication where nodes publish messages to topics and other nodes subscribe to those topics.

### Client-Service (Services)
Synchronous request-response communication where one node provides a service and another calls it.

### Action Servers and Clients
Goal-oriented communication for long-running tasks with feedback.

## Example: Simple Node with Publisher

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisherNode(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Node Management Commands

### Listing Nodes
```bash
ros2 node list
```

### Getting Node Information
```bash
ros2 node info <node_name>
```

### Killing a Node
```bash
ros2 lifecycle set <node_name> shutdown  # For lifecycle nodes
# Or simply use Ctrl+C in the terminal running the node
```

## Node Namespaces

Namespaces help organize nodes in large systems:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        # Node with namespace
        super().__init__('my_node', namespace='robot1')
        # This node will be named '/robot1/my_node'
```

## Advanced Node Features

### Parameters
Nodes can declare and use parameters:

```python
def __init__(self):
    super().__init__('parameter_node')
    self.declare_parameter('my_param', 'default_value')
    my_param = self.get_parameter('my_param').value
```

### Timers
Nodes can create timers for periodic tasks:

```python
def __init__(self):
    super().__init__('timer_node')
    self.timer = self.create_timer(0.5, self.timer_callback)  # 0.5 second period
```

### Quality of Service (QoS)
Configure how messages are delivered:

```python
from rclpy.qos import QoSProfile

qos_profile = QoSProfile(depth=10)
self.publisher_ = self.create_publisher(String, 'topic', qos_profile)
```

## The Role of Nodes in the Robotic Nervous System

In the context of the robotic nervous system:
- **Nodes** act like neurons, processing information and making decisions
- **Topics** act like synapses, enabling communication between nodes
- **Services** act like direct neural pathways for specific requests
- **Parameters** act like genetic information, configuring node behavior

This architecture allows for distributed, modular robot software that can scale from simple single-robot systems to complex multi-robot teams.

## Next Steps

Now that you understand ROS 2 Nodes, continue to learn about:
- **Topics**: How nodes communicate asynchronously
- **Services**: How nodes communicate synchronously
- **URDF**: How to describe robot structure and components