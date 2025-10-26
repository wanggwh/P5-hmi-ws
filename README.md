# Gå til workspace og source environments
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Send error besked (med --once for kun én besked)
ros2 topic pub --once /hmi_error hmi_interface/msg/Error "
stamp:
  sec: 0
  nanosec: 0
severity: 'ERROR'
message: 'hej med dig'
node_name: 'test_node'
"