import select
import sys
import termios
import tty

import rclpy
from cartpole_interfaces.msg import Action
from cartpole_interfaces.srv import SetPosition, Stop
from rclpy.node import Node

class CartpoleTestServer(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('minimal_service')

        self.stop_service_name = '/cartpole/stop'
        self.set_position_service_name = '/cartpole/set_position'

        self.stop_robot_srv_ = self.create_service(Stop, self.stop_service_name, self.stop_robot_callback)
        self.set_position_srv_ = self.create_service(SetPosition, self.set_position_service_name, self.set_position_callback)

    def stop_robot_callback(self, request, response):
        response.success = True
        response.message = "Robot has successfully stopped!"

        self.get_logger().info('Request received from "stop_robot"')
        return response

    def set_position_callback(self, request, response):
        self.get_logger().info('Request received from "set_position"')
        return response


def main():
    node = CartpoleTestServer()

    print("Running test server...")
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()