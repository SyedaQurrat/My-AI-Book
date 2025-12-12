# ROS 2 Communication Contracts

This directory contains the communication contracts for the ROS 2 nervous system examples. These are not traditional API contracts but rather specifications for the ROS 2 message interfaces that will be used in the educational examples.

## Publisher/Subscriber Pattern

### Hello Message Topic
- **Topic Name**: `/hello_world`
- **Message Type**: `std_msgs/msg/String`
- **Publisher Node**: `publisher_node`
- **Subscriber Node**: `subscriber_node`
- **Description**: Simple "Hello World" message publisher/subscriber example

## Service Pattern

### Example Service
- **Service Name**: `/add_two_ints`
- **Request Type**: `example_interfaces/srv/AddTwoInts`
- **Response Type**: `example_interfaces/srv/AddTwoInts`
- **Server Node**: `service_server`
- **Client Node**: `service_client`
- **Description**: Example service that adds two integers

## Node Interfaces

Each example node will follow standard ROS 2 node interface patterns:
- Node lifecycle (init, run, cleanup)
- Parameter handling
- Publisher/subscriber creation and management
- Service/client creation and management