import select
import sys
import termios
import tty

if sys.platform == 'win32':
    import msvcrt

import rclpy
from std_msgs.msg import Float64
from cartpole_interfaces.msg import Action
from cartpole_interfaces.srv import SetPosition, Stop
from rclpy.node import Node

"""
cartpole_teleop_key

Publishes keyboard commands as cartpole/action message
Additionally, implement a client and sends requests when appropriate

Subscribes to:

Publishes to:
    cartpole/action [cartpole_interfaces/msg/Action]             - Desired acceleration of the cart

Sends requests:
    cartpole/stop [cartpole_interfaces/srv/Stop]                 - Stops robot
    cartpole/set_position [cartpole_interfaces/srv/SetPosition]  - Sets absolution position of cart
"""
 
"""
TODO: Implement teleop and services

Useful links:

Teleop:
    1. Generic teleop node example: 
        https://github.com/rohbotics/ros2_teleop_keyboard
        
    2. Refer to cartpole_interfaces/msg for the message formats

Services:
    1. ROS2 services overview:
        https://docs.ros.org/en/foxy/Tutorials/Services/Understanding-ROS2-Services.html

    2. ROS2 services exampl: 
        https://docs.ros.org/en/f
        oxy/Tutorials/Writing-A-Simple-Py-Service-And-Client.html 

    3. Refer to cartpole_interfaces/srv for the service formats
"""

msg = """
Cartpole Teleoperation
----------------------
Moving around:
    a   d

z/x : Increase/Decrease max force
c   : Recenter cart

anything else: stop

CTRL-C to quit
"""

key_bindings = {'a': 1.0,
                'd': -1.0}

def get_key(settings):
    if sys.platform == 'win32':
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def get_terminal_settings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


class CartepoleTeleop(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('cartpole_teleop_key')

        self.topic = '/slider_cart_velocity_controller/command'

        self.stop_service_name = '/cartpole/stop'
        self.set_position_service_name = '/cartpole/set_position'

        self.publisher_ = self.create_publisher(Float64,
                                                self.topic,
                                                10)

        # Clients to send the "Stop" and "Set Position" request
        self.stop_robot_client_ = self.create_client(Stop, self.stop_service_name)
        self.set_position_client_ = self.create_client(SetPosition, self.set_position_service_name)

        # Interfaces used by clients
        self.stop_request = Stop.Request()
        self.set_position_request = SetPosition.Request()

        self.set_position_request.x = 0.0

        #Used to notify of a response from a server
        self.stop_request_future = 0
        self.set_position_request_future = 0

        # Initializes Action interface for publisher
        self.msg = Float64()

        # Limits the value of acceleration
        self.max_accel = 1.0
        self.min_accel = -self.max_accel

        self.delta_accel = 0.1
        self.accel = 0.0

    def publish_accel(self):
        self.msg.data = self.accel

        self.publisher_.publish(self.msg)
        self.get_logger().info('Publishing: "%s" to topic: "%s"' % (self.msg.data, self.topic))

    def limit_accel(self):
        if self.accel > self.max_accel:
            self.accel = self.max_accel
        elif self.accel < self.min_accel:
            self.accel = self.min_accel

    def send_stop_request(self):
        self.stop_request_future = self.stop_robot_client_.call_async(self.stop_request)

    def send_set_position_request(self):
        self.set_position_request_future = self.set_position_client_.call_async(self.set_position_request)


def check_for_service_response(node, future, msg):
    if future.done():
            try:
                response = future.result()
            except Exception as e:
                node.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                node.get_logger().info(msg)


def main():
    settings = get_terminal_settings()
    node = CartepoleTeleop()

    print(msg)

    while True: # Input loop
        key = get_key(settings)
        print(key)

        if key in key_bindings:
            node.accel = key_bindings[key]
        elif key == '\x03':
            break

        node.limit_accel()
        node.publish_accel()
    
    # Sends request to service
    node.send_stop_request()
    node.send_set_position_request()

    # while rclpy.ok(): # Waits for responses from service
    #     rclpy.spin_once(node)

    #     check_for_service_response(node, node.stop_request_future, 'Result of stop_robot')
    #     check_for_service_response(node, node.set_position_request_future, 'Result of set_position')

    #     if node.stop_request_future.done() and node.set_position_request_future.done():
    #         break


    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
