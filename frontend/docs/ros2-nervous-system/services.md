---
title: ROS 2 Services - Synchronous Communication in the Robotic Nervous System
---

# ROS 2 Services - Synchronous Communication in the Robotic Nervous System

## What are ROS 2 Services?

A **Service** is a named interaction where a client node sends a request message and waits for a server node to respond synchronously with a single response message. Services provide synchronous, request-response communication and represent the "direct neural pathways" of the robotic nervous system, enabling specific queries and coordinated actions.

## Key Characteristics of Services

### Synchronous Communication
- Client sends a request and waits for a response
- Server processes the request and sends a response
- Blocking communication until response is received

### One-to-One Pattern
- One service server responds to requests from client nodes
- Each request gets a dedicated response
- Direct connection between client and server during the call

### Request/Response Types
- Each service has a specific request message type
- Each service has a specific response message type
- Types are defined using `.srv` files

## Creating Service Servers and Clients

### Service Definition
First, define the service in a `.srv` file:

```
# AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

This defines:
- Request: two integers `a` and `b`
- Response: one integer `sum`

### Service Server Example
```python
from add_two_ints_client.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()

    try:
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_service.destroy_node()
        rclpy.shutdown()
```

### Service Client Example
```python
from add_two_ints_client.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    response = minimal_client.send_request(2, 3)
    minimal_client.get_logger().info(f'Result of add_two_ints: {response.sum}')
    minimal_client.destroy_node()
    rclpy.shutdown()
```

## Service Commands

### Listing Services
```bash
ros2 service list
```

### Getting Service Information
```bash
ros2 service info /add_two_ints
```

### Calling a Service (Command Line)
```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
```

## Service Types

### Common Built-in Service Types
- `std_srvs`: Basic services (Empty, Trigger, SetBool, etc.)
- `example_interfaces`: Example services (AddTwoInts, etc.)
- Custom services defined in your packages

### Creating Custom Service Types
Custom service types are defined in `.srv` files in the `srv/` directory of a package:

```
# Custom service example: ComputePath.srv
geometry_msgs/Point start
geometry_msgs/Point goal
---
nav_msgs/Path path
bool success
string error_message
```

The request and response parts are separated by `---`.

## Advanced Service Features

### Service Callback Groups
Control execution of service callbacks:

```python
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# Create a callback group
cb_group = MutuallyExclusiveCallbackGroup()

# Use the callback group when creating the service
self.srv = self.create_service(
    AddTwoInts,
    'add_two_ints',
    self.add_two_ints_callback,
    callback_group=cb_group
)
```

### Service Quality of Service
Services have their own QoS settings:

```python
from rclpy.qos import QoSProfile

# Services use specific QoS for service communication
self.srv = self.create_service(
    AddTwoInts,
    'add_two_ints',
    self.add_two_ints_callback
)
```

### Service Error Handling
Handle service call failures:

```python
def send_request(self, a, b):
    self.req.a = a
    self.req.b = b
    self.future = self.cli.call_async(self.req)

    try:
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=5.0)
        if self.future.done():
            return self.future.result()
        else:
            self.get_logger().error('Service call timed out')
            return None
    except Exception as e:
        self.get_logger().error(f'Service call failed: {e}')
        return None
```

## Service vs. Topic Comparison

| Feature | Topics | Services |
|---------|--------|----------|
| Communication | Asynchronous | Synchronous |
| Pattern | Many-to-many | One-to-one |
| Latency | Low (fire and forget) | Higher (wait for response) |
| Reliability | Best effort or reliable | Reliable |
| Use Case | Streaming data | Specific queries/commands |

## Service Performance Considerations

### Latency
- Services block until response is received
- Consider using topics for high-frequency data
- Design services to respond quickly

### Concurrency
- Multiple clients can call the same service
- Service server processes requests sequentially by default
- Consider using multiple callback groups for concurrency

### Error Handling
- Always handle service call failures
- Implement timeouts to prevent indefinite blocking
- Design services to handle errors gracefully

## The Role of Services in the Robotic Nervous System

Services function as the synchronous communication mechanism in the robotic nervous system:

- **Configuration**: Request specific configuration changes
- **Activation**: Turn on/off specific behaviors or components
- **Queries**: Request specific information that requires processing
- **Coordination**: Synchronize actions between nodes

This synchronous communication pattern allows for reliable, coordinated robot behaviors where nodes need to wait for confirmation before proceeding.

## Common Service Patterns

### Parameter Configuration
Change parameters of a node or system component.

### Activation/Deactivation
Turn on/off specific robot behaviors or subsystems.

### Data Queries
Request specific processed data that requires computation.

### Action Coordination
Coordinate multi-step processes that require acknowledgment.

## Service Best Practices

### Response Time
- Keep service responses fast
- For long computations, consider using Actions instead
- Provide feedback if processing will take time

### Error Handling
- Handle all possible error conditions
- Provide meaningful error messages
- Use appropriate return codes

### Robustness
- Handle multiple simultaneous requests
- Validate input parameters
- Clean up resources properly

## Next Steps

Now that you understand ROS 2 Services, continue to learn about:
- **URDF**: How to describe robot structure and components
- **Architecture**: How nodes, topics, and services work together in the ROS 2 graph
- **Hello World Tutorial**: Practical implementation of publisher/subscriber patterns