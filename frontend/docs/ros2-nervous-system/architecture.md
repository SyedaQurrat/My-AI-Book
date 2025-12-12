---
title: ROS 2 Architecture - The Graph and Communication Patterns
---

# ROS 2 Architecture - The Graph and Communication Patterns

## The ROS 2 Graph Architecture

The ROS 2 graph represents the dynamic network of nodes, topics, services, and actions that form the robotic nervous system. Understanding this architecture is crucial for designing effective robot software systems.

## Core Architecture Components

### Nodes
- **Role**: The fundamental computational units
- **Function**: Execute specific tasks and communicate with other nodes
- **Characteristics**: Process-based, single-threaded by default, can contain multiple publishers/subscribers/services

### Topics (Publishers and Subscribers)
- **Role**: Asynchronous data distribution
- **Function**: Enable many-to-many communication through message passing
- **Characteristics**: Non-blocking, decoupled in time and space, supports Quality of Service (QoS) policies

### Services (Clients and Servers)
- **Role**: Synchronous request-response communication
- **Function**: Enable direct queries and coordinated actions
- **Characteristics**: Blocking calls, one-to-one communication, request-response pattern

### Actions
- **Role**: Goal-oriented communication for long-running tasks
- **Function**: Enable tasks with feedback and status updates
- **Characteristics**: Goal-feedback-result pattern, cancellable, status tracking

## The ROS 2 Communication Model

### Distributed Architecture
ROS 2 uses a distributed architecture based on the Data Distribution Service (DDS) middleware:

```
Node A          Node B          Node C
  |               |               |
  | Publishes     | Subscribes    | Provides
  | to topic X    | to topic X    | Service Y
  |               |               |
  |<------------ Topic X ---------->|
  |               |               |
  | Calls         | Responds      |
  | Service Y     | to Service Y  |
  |-------------->|<--------------|
```

### Masterless Design
Unlike ROS 1, ROS 2 has no central master node:
- Nodes discover each other automatically
- Communication is peer-to-peer
- More robust and scalable

## Quality of Service (QoS) in the Architecture

QoS policies define how data is communicated between nodes:

### Reliability
- **Reliable**: All messages are guaranteed to be delivered
- **Best Effort**: Messages may be lost, but with higher throughput

### Durability
- **Transient Local**: Late-joining subscribers receive the last message
- **Volatile**: No messages are stored for late joiners

### History
- **Keep Last**: Store the most recent N messages
- **Keep All**: Store all messages (use with caution)

### Lifespan and Deadline
- **Lifespan**: How long messages remain valid
- **Deadline**: Expected frequency of data publication

## Design Patterns in ROS 2 Architecture

### Publisher-Subscriber Pattern
```
Sensor Node → Topic → Multiple Processing Nodes
     ↓           ↓           ↓
  Camera    → image_raw → Perception, Logging, GUI
```

### Client-Server Pattern
```
Navigation Node → Service → Map Server
     ↓              ↓           ↓
  Request Map   → get_map  → Return Map Data
```

### Action Pattern
```
MoveIt Node → Action → Joint Controller
     ↓           ↓           ↓
  Goal: Move → move_joints → Feedback: 50% complete
```

## Node Organization Strategies

### Functional Organization
Group nodes by function:
- Perception nodes (sensors, computer vision)
- Planning nodes (path planning, motion planning)
- Control nodes (motor control, actuator interfaces)

### Hierarchical Organization
Organize nodes in a hierarchy:
- Supervisor nodes coordinate multiple worker nodes
- Top-level nodes manage system state
- Bottom-level nodes handle hardware interfaces

### Component Organization
Group related functionality:
- All components for a specific sensor
- All components for a specific actuator
- All components for a specific behavior

## Communication Design Principles

### Decoupling
- Nodes should be independent of each other
- Use topics for loose coupling
- Minimize direct dependencies

### Scalability
- Design systems that can handle multiple instances
- Use namespaces for multiple robots
- Consider network bandwidth and latency

### Robustness
- Handle node failures gracefully
- Use appropriate QoS settings
- Implement error recovery

### Performance
- Minimize unnecessary message passing
- Use appropriate message rates
- Consider data compression for large messages

## The ROS 2 Middleware Layer

### DDS (Data Distribution Service)
- **Role**: Provides the underlying communication infrastructure
- **Function**: Handles message routing, discovery, and delivery
- **Benefits**: Language agnostic, supports multiple vendors, real-time capable

### RMW (ROS Middleware)
- **Role**: Abstracts DDS implementation details
- **Function**: Provides ROS 2 API to DDS functionality
- **Benefits**: Allows switching between DDS implementations

### Client Libraries (rclpy, rclcpp)
- **Role**: Provide language-specific APIs
- **Function**: Interface between application code and middleware
- **Benefits**: Familiar language patterns, type safety

## Architecture Visualization Tools

### Command Line Tools
```bash
# View the current ROS graph
ros2 graph

# List all nodes
ros2 node list

# List all topics
ros2 topic list

# List all services
ros2 service list

# Show node connections
ros2 node info <node_name>
```

### GUI Tools
- **RViz2**: 3D visualization of robot state and sensor data
- **rqt_graph**: Visual representation of the ROS graph
- **rqt_console**: Real-time logging information

## Architecture Best Practices

### System Design
- Start with a clear system architecture diagram
- Identify all required nodes and their responsibilities
- Plan topic and service interfaces early

### Interface Design
- Use standard message types when possible
- Design clear, consistent interfaces
- Document all interfaces thoroughly

### Resource Management
- Consider computational resources of target platform
- Plan for memory usage in embedded systems
- Account for network bandwidth limitations

### Testing and Validation
- Design for testability from the beginning
- Use simulation for early validation
- Plan integration testing strategies

## The Robotic Nervous System Architecture

In the context of the robotic nervous system:

- **Nodes** act like neurons, processing information and making decisions
- **Topics** act like synapses, enabling asynchronous communication
- **Services** act like direct neural pathways for specific requests
- **Actions** act like complex motor commands with feedback
- **Parameters** act like genetic information, configuring system behavior
- **URDF** acts like the skeleton, defining the physical structure

This architecture enables the development of complex, distributed robot software systems that can scale from simple single-robot applications to complex multi-robot systems.

## Architecture Considerations for Jetson Orin Nano

### Resource Constraints
- Optimize for CPU and memory usage
- Consider power consumption implications
- Use appropriate QoS settings for real-time requirements

### Real-time Performance
- Configure DDS for deterministic behavior
- Use appropriate scheduling policies
- Minimize communication latency

### Safety and Security
- Implement proper error handling
- Use secure communication when needed
- Plan for safe failure modes

## Next Steps

Now that you understand the ROS 2 architecture, continue to learn about:
- **Hello World Tutorial**: Practical implementation of publisher/subscriber patterns
- **Security**: Best practices for securing ROS 2 communications
- **Performance**: Optimization techniques for embedded systems