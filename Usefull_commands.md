## For at simulere en Error besked, skriv denne kommando i terminalen (Efter at have sourced dit ws)
ros2 topic pub /error_messages p5_interfaces/msg/Error "{stamp: {sec: 0, nanosec: 0}, severity: 'FATAL', message: 'Jensens kode fejler fuldstændig', node_name: 'test_node'}" --once
