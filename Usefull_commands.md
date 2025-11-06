## For at simulere en Error besked, skriv denne kommando i terminalen (Efter at have sourced dit ws)
ros2 topic pub /error_messages p5_interfaces/msg/Error "{stamp: {sec: 0, nanosec: 0}, severity: 'FATAL', message: 'Jensens kode er fantastisk men jeg Gustav har ødelagt alt', node_name: 'test_node'}" --once
