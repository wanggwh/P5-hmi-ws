from p5_interfaces.srv import MoveToPreDefPose 

import rclpy
from rclpy.node import Node


class ServiceNode(Node):

    def __init__(self):
        super().__init__('service_node')
        self.srv = self.create_service(MoveToPreDefPose, 'p5_move_to_pre_def_pose', self.robot_configurations_callback)
        self.get_logger().info('Service node for status popup dialog testing started.')

    def robot_configurations_callback(self, request, response):
        request.robot_name
        request.goal_name
        response.success = True
        self.get_logger().info('Received request for robot: %s, goal: %s' % (request.robot_name, request.goal_name) + 
                               " Resonded with status: " + str(response.success))

        return response


def main():
    rclpy.init()

    service_node = ServiceNode()

    rclpy.spin(service_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()