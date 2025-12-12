---
title: Hello World Tutorial - Publisher/Subscriber in Python
---

# Hello World Tutorial - Publisher/Subscriber in Python

This tutorial will guide you through creating your first ROS 2 publisher and subscriber nodes in Python. This is the foundational "Hello World" example that demonstrates the basic communication pattern in ROS 2.

## Prerequisites

Before starting this tutorial, ensure you have:
- ROS 2 Humble Hawksbill installed
- Python 3.8+ available
- Basic Python programming knowledge
- Completed the prerequisites setup

## Creating the Package

First, create a new ROS 2 package for your examples:

```bash
# Create a workspace if you don't have one
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create the package
ros2 pkg create --build-type ament_python py_pubsub --dependencies rclpy std_msgs
```

## Creating the Publisher Node

Create the publisher node in `~/ros2_ws/src/py_pubsub/py_pubsub/publisher_member_function.py`:

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

## Creating the Subscriber Node

Create the subscriber node in `~/ros2_ws/src/py_pubsub/py_pubsub/subscriber_member_function.py`:

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

## Configuring the Package

Update the `setup.py` file in `~/ros2_ws/src/py_pubsub/setup.py` to include entry points:

```python
from setuptools import find_packages
from setuptools import setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Python pubsub example',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.publisher_member_function:main',
            'listener = py_pubsub.subscriber_member_function:main',
        ],
    },
)
```

## Building the Package

Navigate back to your workspace and build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select py_pubsub
```

## Sourcing the Environment

After building, source the setup file:

```bash
source ~/ros2_ws/install/setup.bash
```

## Running the Example

Open two separate terminals:

**Terminal 1** (Publisher):
```bash
source ~/ros2_ws/install/setup.bash
ros2 run py_pubsub talker
```

**Terminal 2** (Subscriber):
```bash
source ~/ros2_ws/install/setup.bash
ros2 run py_pubsub listener
```

You should see the publisher sending "Hello World" messages with incrementing numbers, and the subscriber receiving and printing these messages.

## Understanding the Code

### Publisher Code Breakdown

```python
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        # Create a publisher that sends String messages to the 'topic' topic
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        # Create a timer that calls the callback every 0.5 seconds
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0  # Counter for messages

    def timer_callback(self):
        msg = String()  # Create a message
        msg.data = 'Hello World: %d' % self.i  # Set message content
        self.publisher_.publish(msg)  # Publish the message
        self.get_logger().info('Publishing: "%s"' % msg.data)  # Log to console
        self.i += 1  # Increment counter
```

### Subscriber Code Breakdown

```python
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        # Create a subscription to the 'topic' topic with String messages
        self.subscription = self.create_subscription(
            String,
            'topic',              # Topic name
            self.listener_callback,  # Callback function
            10)                   # QoS history depth
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        # This function is called when a message is received
        self.get_logger().info('I heard: "%s"' % msg.data)
```

## Modifying the Example

### Changing the Topic Name
You can change the topic name by modifying both the publisher and subscriber:

```python
# In both files, change:
self.publisher_ = self.create_publisher(String, 'my_custom_topic', 10)
# and
self.create_subscription(String, 'my_custom_topic', self.listener_callback, 10)
```

### Changing the Message Rate
Modify the `timer_period` in the publisher to change how frequently messages are sent:

```python
timer_period = 1.0  # Change from 0.5 to 1.0 seconds
```

### Adding Custom Message Content
Modify the message content in the publisher:

```python
import datetime

def timer_callback(self):
    msg = String()
    current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    msg.data = f'Hello World at {current_time}: {self.i}'
    self.publisher_.publish(msg)
    self.get_logger().info(f'Publishing: "{msg.data}"')
    self.i += 1
```

## Troubleshooting Common Issues

### "Command 'ros2' not found"
Make sure you have sourced your ROS 2 environment:
```bash
source /opt/ros/humble/setup.bash
```

### Nodes can't communicate
- Verify both nodes are on the same ROS domain: `echo $ROS_DOMAIN_ID`
- Check that topic names match exactly
- Ensure you sourced the setup file after building: `source ~/ros2_ws/install/setup.bash`

### Package doesn't build
- Check that all dependencies are installed
- Verify the setup.py file is correctly formatted
- Make sure you're in the correct workspace directory

## Advanced Concepts in the Example

### Quality of Service (QoS)
The number `10` in the publisher and subscriber creation represents the history depth - how many messages to keep in the queue.

### Timers
The publisher uses a timer to send messages at regular intervals, which is a common pattern for sensor data or status updates.

### Logging
The `get_logger().info()` method provides console output for debugging and monitoring.

## Extending the Example

### Adding Services
You could extend this example by adding a service that allows external nodes to reset the counter:

```python
from std_srvs.srv import Empty

class MinimalPublisher(Node):
    def __init__(self):
        # ... existing code ...
        self.reset_service = self.create_service(
            Empty, 'reset_counter', self.reset_counter_callback)

    def reset_counter_callback(self, request, response):
        self.i = 0
        self.get_logger().info('Counter reset to 0')
        return response
```

### Adding Parameters
Add runtime configuration:

```python
def __init__(self):
    # ... existing code ...
    self.declare_parameter('message_prefix', 'Hello World')
    self.message_prefix = self.get_parameter('message_prefix').value

def timer_callback(self):
    msg = String()
    msg.data = f'{self.message_prefix}: {self.i}'
    # ... rest of the code ...
```

## Running on Jetson Orin Nano

To run this example on the Jetson Orin Nano:

1. Ensure ROS 2 Humble is installed on the Jetson
2. Copy your workspace to the Jetson or develop directly on it
3. Install any required dependencies
4. Build the package: `colcon build --packages-select py_pubsub`
5. Source and run as described above

## Next Steps

Now that you've successfully created and run your first ROS 2 publisher/subscriber pair:

1. Try modifying the example to send different types of messages
2. Create a service client/server example
3. Explore more complex message types
4. Learn about ROS 2 launch files to run multiple nodes together
5. Continue to the troubleshooting guide for more advanced techniques