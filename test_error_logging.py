#!/usr/bin/env python3
"""
Test script for error logging functionality
"""
import rclpy
from rclpy.node import Node
from p5_interfaces.msg import Error
import time

def test_error_logging():
    rclpy.init()
    node = Node('test_error_publisher')
    publisher = node.create_publisher(Error, '/error_messages', 10)
    
    # Wait for publisher to be ready
    time.sleep(2)
    
    # Test different error severities
    errors = [
        ('ERROR', 'Test error message 1', 'test_node'),
        ('WARNING', 'Test warning message 2', 'test_node'),
        ('INFO', 'Test info message 3', 'test_node'),
        ('FATAL', 'Test fatal message 4', 'test_node'),
        ('INFO', 'Test debug message 5', 'test_node'),
        ('WARNING', 'Another warning message 6', 'test_node'),
        ('ERROR', 'Another error message 7', 'test_node'),
        ('FATAL', 'Critical failure message 8', 'test_node'),
        ('INFO', 'Informational message 9', 'test_node')
    ]
    
    print('Publishing test error messages...')
    for i, (severity, message, node_name) in enumerate(errors):
        error_msg = Error()
        error_msg.severity = severity
        error_msg.message = f"{message} - #{i+1}"
        error_msg.node_name = node_name
        
        publisher.publish(error_msg)
        print(f'Published: [{severity}] from {node_name}: {message} - #{i+1}')
        time.sleep(2)  # Wait 2 seconds between messages
    
    print('Test complete - check HMI for error messages in System Logging section')
    rclpy.shutdown()

if __name__ == '__main__':
    test_error_logging()