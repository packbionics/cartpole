import sys
#sys.path.append('/home/anthony/dev_ws/install/cartpole_control/lib/python3.8/site-packages/cartpole_control')

import rclpy
import threading

from cartpole_control.cart_pole_control import CartPoleEnergyShapingController

def publish(_controller):
    while True:
        _controller.publish_vel()

def main():
    controller = CartPoleEnergyShapingController()

    t1 = threading.Thread(target=publish, args=[controller])
    t1.start()

    rclpy.spin(controller)

    t1.join()

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()