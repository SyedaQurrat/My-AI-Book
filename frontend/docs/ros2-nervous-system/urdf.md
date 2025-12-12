---
title: URDF - Unified Robot Description Format
---

# URDF - Unified Robot Description Format

## What is URDF?

**URDF (Unified Robot Description Format)** is an XML file format used in ROS to describe the physical and kinematic properties of a robot, including its joints, links, and sensors. URDF serves as the "DNA" of the robotic nervous system, defining the physical structure that the software components will control and interact with.

## Key Components of URDF

### Links
- **Definition**: Physical components of the robot (e.g., base, arms, wheels)
- **Properties**: Mass, inertia, visual appearance, collision properties
- **Structure**: Each link represents a rigid body part of the robot

### Joints
- **Definition**: Connections between links that allow relative motion
- **Types**: Fixed, continuous, revolute, prismatic, floating, planar
- **Properties**: Limits, dynamics, safety controllers

### Materials
- **Definition**: Visual appearance properties (color, texture)
- **Usage**: Define how the robot appears in simulation and visualization tools

## Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Define materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>

  <!-- Define links -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Define joints -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Link Properties

### Visual Properties
- **Geometry**: Shape (box, cylinder, sphere, mesh)
- **Material**: Color and texture
- **Origin**: Position and orientation relative to joint

### Collision Properties
- **Geometry**: Shape for collision detection
- **Can be different from visual geometry** for performance reasons

### Inertial Properties
- **Mass**: Physical mass of the link
- **Inertia**: Moment of inertia tensor values
- **Critical for physics simulation**

## Joint Types

### Fixed Joint
- No movement allowed between parent and child links
- Used for attaching static components

```xml
<joint name="fixed_joint" type="fixed">
  <parent link="base_link"/>
  <child link="sensor_link"/>
</joint>
```

### Revolute Joint
- Single axis rotation with limits
- Common for rotational joints like elbows or knees

```xml
<joint name="revolute_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="lower_arm"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

### Continuous Joint
- Unlimited rotation around single axis
- Common for wheel joints

```xml
<joint name="continuous_joint" type="continuous">
  <parent link="base_link"/>
  <child link="wheel"/>
  <axis xyz="0 1 0"/>
</joint>
```

### Prismatic Joint
- Linear sliding motion along single axis

```xml
<joint name="prismatic_joint" type="prismatic">
  <parent link="base_link"/>
  <child link="slide"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="0.5" effort="100" velocity="1"/>
</joint>
```

## URDF Tools and Commands

### Checking URDF Files
```bash
# Validate URDF syntax
check_urdf my_robot.urdf

# View robot model
urdf_to_graphiz my_robot.urdf
```

### Visualizing URDF
```bash
# Launch RViz with robot model
ros2 run rviz2 rviz2
# Add RobotModel display and set robot description to your URDF
```

### Converting to/from XACRO
XACRO is a macro language that simplifies complex URDF files:

```xml
<!-- XACRO example -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">

  <xacro:property name="M_PI" value="3.1415926535897931" />

  <xacro:macro name="cylinder_inertial" params="mass radius length">
    <inertial>
      <mass value="${mass}" />
      <inertia ixx="${0.0833333 * mass * (3 * radius * radius + length * length)}"
               ixy="0" ixz="0"
               iyy="${0.0833333 * mass * (3 * radius * radius + length * length)}"
               iyz="0"
               izz="${0.5 * mass * radius * radius}" />
    </inertial>
  </xacro:macro>

  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.2" length="0.6"/>
      </geometry>
    </visual>
    <xacro:cylinder_inertial mass="10" radius="0.2" length="0.6"/>
  </link>
</robot>
```

## URDF in Simulation

### Gazebo Integration
URDF files can be extended with Gazebo-specific tags:

```xml
<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>

<!-- Gazebo plugins -->
<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
  </plugin>
</gazebo>
```

### Joint State Publisher
For visualization without simulation:

```xml
<joint name="example_joint" type="revolute">
  <parent link="base_link"/>
  <child link="moving_part"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

## Best Practices for URDF

### Organization
- Use descriptive names for links and joints
- Group related components logically
- Maintain a consistent naming convention

### Validation
- Always validate URDF files before use
- Check for proper joint limits and dynamics
- Verify mass and inertia properties

### Performance
- Use simple collision geometries when possible
- Avoid overly complex meshes
- Consider using separate URDF files for different purposes (visualization vs. collision)

### Simulation Accuracy
- Accurate mass and inertia properties are crucial for physics simulation
- Include realistic joint limits and dynamics
- Consider using transmission elements for actuator modeling

## URDF and the Robotic Nervous System

URDF connects the physical and software layers of the robotic nervous system:

- **Hardware Abstraction**: Provides a standardized way to describe robot structure
- **Simulation**: Enables physics simulation for testing and development
- **Visualization**: Allows tools like RViz to display robot models
- **Control**: Provides kinematic information needed for motion planning

The URDF model serves as the foundation upon which the ROS 2 nodes, topics, and services operate, defining the physical structure that the software components will control and interact with.

## Common URDF Issues and Solutions

### Self-Collision
- Use collision tags carefully to avoid false positives
- Adjust collision meshes for performance vs. accuracy

### Joint Limits
- Set realistic joint limits to prevent damage
- Include safety margins in limit definitions

### Mass Properties
- Use realistic mass and inertia values
- Verify center of mass is correctly positioned

## Next Steps

Now that you understand URDF, continue to learn about:
- **Architecture**: How nodes, topics, services, and URDF work together in the ROS 2 graph
- **Hello World Tutorial**: Practical implementation of publisher/subscriber patterns
- **Security**: Best practices for securing ROS 2 communications