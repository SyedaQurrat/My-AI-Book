# Data Model: Module 1: The Robotic Nervous System (ROS 2)

## Overview
This document describes the key entities and data structures for the ROS 2 nervous system module. Since this is primarily an educational module, the "data model" focuses on the conceptual entities that students need to understand.

## Entity: ROS 2 Node
- **Description**: An executable process that performs computation, often a single-purpose component in the robotic system
- **Attributes**:
  - `name`: String - Unique identifier for the node within the ROS graph
  - `namespace`: String - Optional namespace for organizing nodes
  - `parameters`: Dictionary - Configuration values for the node
  - `publishers`: List - Publishers created by this node
  - `subscribers`: List - Subscribers created by this node
  - `services`: List - Services provided by this node
  - `clients`: List - Service clients created by this node
- **Relationships**: Can publish to Topics, subscribe to Topics, provide Services, and call Services
- **State Transitions**: [Not applicable for this conceptual entity]

## Entity: ROS 2 Topic
- **Description**: A named bus over which nodes exchange messages asynchronously, forming a many-to-many communication pattern
- **Attributes**:
  - `name`: String - Unique identifier for the topic
  - `message_type`: String - Type of message being published (e.g., std_msgs/String)
  - `publishers`: List - Nodes publishing to this topic
  - `subscribers`: List - Nodes subscribed to this topic
- **Relationships**: Connects Nodes in a publisher/subscriber pattern
- **State Transitions**: [Not applicable for this conceptual entity]

## Entity: ROS 2 Service
- **Description**: A named interaction where a client node sends a request message and waits for a server node to respond synchronously with a single response message
- **Attributes**:
  - `name`: String - Unique identifier for the service
  - `request_type`: String - Type of request message
  - `response_type`: String - Type of response message
  - `server`: Node - The node providing the service
  - `clients`: List - Nodes that call this service
- **Relationships**: Connects service client Nodes to service server Nodes
- **State Transitions**: [Not applicable for this conceptual entity]

## Entity: URDF (Unified Robot Description Format)
- **Description**: An XML file format used in ROS to describe the physical and kinematic properties of a robot, including its joints, links, and sensors
- **Attributes**:
  - `robot_name`: String - Name of the robot
  - `links`: List - Physical components of the robot (e.g., base, arms)
  - `joints`: List - Connections between links
  - `materials`: List - Visual materials for rendering
  - `gazebo_extensions`: List - Simulation-specific extensions
- **Relationships**: Defines the physical structure of a robot for simulation and real-world applications
- **State Transitions**: [Not applicable for this conceptual entity]

## Validation Rules from Requirements
- All ROS 2 entities must have unique names within their namespace
- Message types must be properly defined and compatible between publishers and subscribers
- Service request/response types must match between client and server
- URDF files must be valid XML and follow ROS conventions

## Relationships
- Nodes can have multiple Publishers and Subscribers to different Topics
- Nodes can provide multiple Services and act as clients for multiple Services
- Topics enable many-to-many communication between Nodes
- Services enable one-to-one synchronous communication between Nodes
- URDF describes the physical structure that Nodes may control or interact with
