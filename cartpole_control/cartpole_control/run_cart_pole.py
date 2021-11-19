import sys


import rclpy
from cartpole_control.controllers import CartPoleEnergyShapingController

def publish(_controller):
    while True:
        _controller.publish_vel()

def main():

    controller = CartPoleEnergyShapingController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()