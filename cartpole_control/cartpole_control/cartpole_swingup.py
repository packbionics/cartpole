import rclpy

from cartpole_control.controllers import CartPoleEnergyShapingController
from cartpole_control.controllers import CartPoleMPCController

def main():
    rclpy.init()
    #controller = CartPoleEnergyShapingController()
    controller = CartPoleMPCController()
    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
