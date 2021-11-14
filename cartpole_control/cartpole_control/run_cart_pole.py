import sys
sys.path.append('/home/anthony/dev_ws/install/cartpole_control/lib/python3.8/site-packages/cartpole_control')

import rclpy
import threading

from cart_pole_control import CartPoleEnergyShapingController

def publish_effort(_controller):
    while True:
        _controller.publish_effort()

def main():
    controller = CartPoleEnergyShapingController()

    #t1 = threading.Thread(target=publish_effort, args=[controller])
    #t1.start()

    rclpy.spin(controller)

    #t1.join()

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()