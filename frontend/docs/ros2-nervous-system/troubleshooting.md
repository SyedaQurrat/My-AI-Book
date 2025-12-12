---
title: Troubleshooting Common ROS 2 Issues
---

# Troubleshooting Common ROS 2 Issues

This guide covers common issues you may encounter when working with ROS 2 and provides solutions to help you resolve them quickly.

## Environment and Setup Issues

### "Command 'ros2' not found"
**Problem**: The `ros2` command is not recognized in your terminal.

**Solutions**:
1. **Source the ROS 2 environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Add to your shell profile** to make it permanent:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

3. **Check if ROS 2 is properly installed**:
   ```bash
   ls /opt/ros/humble/
   ```

### Python Import Errors
**Problem**: Getting import errors when trying to use ROS 2 Python libraries.

**Solutions**:
1. **Check your Python environment**:
   ```bash
   python3 -c "import rclpy; print('rclpy imported successfully')"
   ```

2. **Install Python packages if needed**:
   ```bash
   pip3 install ros-humble-rclpy ros-humble-std-msgs
   ```

3. **Use the correct Python version** (3.8+ for Humble).

## Communication Issues

### Nodes Can't Communicate
**Problem**: Publisher and subscriber nodes can't see each other's messages.

**Solutions**:
1. **Check that both nodes are on the same ROS domain**:
   ```bash
   echo $ROS_DOMAIN_ID
   # Set if needed: export ROS_DOMAIN_ID=42
   ```

2. **Verify topic names match exactly**:
   ```bash
   ros2 topic list  # Check what topics exist
   ros2 topic info /your_topic_name  # Check topic details
   ```

3. **Check network configuration**:
   ```bash
   export ROS_LOCALHOST_ONLY=0  # If communicating across machines
   ```

4. **Verify QoS compatibility** between publisher and subscriber.

### Services Not Responding
**Problem**: Service clients timeout waiting for responses.

**Solutions**:
1. **Ensure service server is running**:
   ```bash
   ros2 service list  # Check if service exists
   ```

2. **Verify service name matches exactly**.

3. **Check for blocking operations** in the service callback.

## Build and Package Issues

### Package Doesn't Build
**Problem**: `colcon build` fails with errors.

**Solutions**:
1. **Check dependencies are installed**:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. **Verify package.xml dependencies** match your setup.py requirements.

3. **Clean build artifacts** and rebuild:
   ```bash
   rm -rf build/ install/ log/
   colcon build
   ```

4. **Check for syntax errors** in Python files before building.

### Module Not Found After Build
**Problem**: Getting "Module not found" errors after building.

**Solutions**:
1. **Source the install setup**:
   ```bash
   source install/setup.bash
   ```

2. **Verify entry points in setup.py** are correctly defined.

3. **Check the package structure** matches ROS 2 conventions.

## Performance Issues

### High CPU Usage
**Problem**: ROS 2 nodes consume excessive CPU resources.

**Solutions**:
1. **Reduce message publishing rates** in timers.

2. **Use appropriate QoS settings**:
   ```python
   # Reduce history depth if you don't need old messages
   qos_profile = QoSProfile(depth=1)  # Instead of 10
   ```

3. **Optimize callback functions** to be lightweight.

### Memory Leaks
**Problem**: Memory usage grows continuously.

**Solutions**:
1. **Properly destroy nodes** in shutdown:
   ```python
   node.destroy_node()
   rclpy.shutdown()
   ```

2. **Check for circular references** in your code.

3. **Monitor with system tools**:
   ```bash
   htop  # Or other system monitoring tools
   ```

## Jetson Orin Nano Specific Issues

### Resource Constraints
**Problem**: Limited CPU/RAM on Jetson platform.

**Solutions**:
1. **Optimize for ARM64 architecture** when building.

2. **Reduce node complexity** or number of concurrent nodes.

3. **Monitor system resources**:
   ```bash
   nvidia-smi  # Check GPU usage
   free -h     # Check RAM usage
   top         # Check CPU usage
   ```

### Power Management
**Problem**: Node performance affected by power states.

**Solutions**:
1. **Set Jetson to maximum performance mode**:
   ```bash
   sudo nvpmodel -m 0  # Maximum performance
   sudo jetson_clocks  # Lock clocks to maximum
   ```

## Debugging Techniques

### Using ROS 2 Command Line Tools
```bash
# List all active nodes
ros2 node list

# Get information about a specific node
ros2 node info /node_name

# List all topics
ros2 topic list

# Echo messages from a topic
ros2 topic echo /topic_name MessageType

# Check topic statistics
ros2 topic hz /topic_name

# Call a service
ros2 service call /service_name ServiceType "{request: data}"
```

### Logging and Debugging
```python
# In your nodes, use proper logging
self.get_logger().debug('Debug message')
self.get_logger().info('Info message')
self.get_logger().warn('Warning message')
self.get_logger().error('Error message')
self.get_logger().fatal('Fatal message')
```

### Using RViz2 for Visualization
```bash
# Launch RViz2 for visual debugging
ros2 run rviz2 rviz2
```

## Common Error Messages and Solutions

### "No executable found"
**Error**: `ros2 run package_name executable_name` fails.

**Solution**:
1. Check that you've sourced the install setup: `source install/setup.bash`
2. Verify the executable name matches your entry point in setup.py
3. Ensure the package was built successfully

### "Topic/Service already exists"
**Error**: Getting warnings about existing topics/services.

**Solution**: This is usually not a problem, but if you need clean state:
```bash
# Restart the daemon to clear old registrations
pkill -f ros
# Or set a different domain ID
export ROS_DOMAIN_ID=43
```

### "Failed to load entry point"
**Error**: When running a node, getting "Failed to load entry point" error.

**Solution**:
1. Check your setup.py entry points format
2. Verify the module and function exist
3. Check for Python syntax errors in your files

## Network Troubleshooting

### Multi-Machine Communication
**Problem**: Nodes on different machines can't communicate.

**Solutions**:
1. **Ensure same ROS_DOMAIN_ID on all machines**:
   ```bash
   export ROS_DOMAIN_ID=42
   ```

2. **Check firewall settings** to allow ROS 2 traffic (DDS uses various ports).

3. **Verify network connectivity**:
   ```bash
   ping other_machine_ip
   ```

4. **Set appropriate ROS_LOCALHOST_ONLY**:
   ```bash
   export ROS_LOCALHOST_ONLY=0
   ```

## Debugging Tools

### Built-in ROS 2 Tools
- `rqt_graph`: Visualize the ROS graph
- `rqt_console`: View log messages
- `rqt_plot`: Plot numeric values over time
- `ros2 doctor`: Check system configuration

### System Monitoring
```bash
# Monitor ROS 2 processes
htop -p $(pgrep -f ros2)

# Check network usage
netstat -tuln | grep -i ros

# Monitor disk space
df -h
```

## Best Practices for Avoiding Issues

### Code Quality
1. **Always handle exceptions** in callbacks
2. **Use proper resource management** (cleanup nodes, close files)
3. **Validate inputs** before processing
4. **Use appropriate logging levels**

### Development Workflow
1. **Test in simulation first** before running on hardware
2. **Use version control** to track changes
3. **Write unit tests** for critical functionality
4. **Document your code** and interfaces clearly

## Getting Help

### When to Seek Help
- Issues persist after trying multiple solutions
- Unclear error messages
- Performance problems affecting system operation

### Resources
- **ROS 2 Documentation**: https://docs.ros.org/
- **ROS Answers**: https://answers.ros.org/
- **ROS Discourse**: https://discourse.ros.org/
- **GitHub Issues**: For specific packages

## Quick Reference Commands

```bash
# Essential troubleshooting commands
ros2 node list                    # List all nodes
ros2 topic list                   # List all topics
ros2 service list                 # List all services
ros2 action list                  # List all actions
ros2 param list /node_name        # List node parameters
ros2 doctor                       # Check system health

# Environment check
printenv | grep ROS               # Check ROS environment variables
source /opt/ros/humble/setup.bash # Source ROS environment
```

## Next Steps

After resolving your immediate issues:

1. Review the security and performance considerations for production use
2. Learn about advanced ROS 2 features like Actions and Parameters
3. Explore integration with the Jetson Orin Nano hardware
4. Consider using launch files to manage complex multi-node systems