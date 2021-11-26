import select
import sys
import termios
import threading
import time
import tty

if sys.platform == 'win32':
    import msvcrt

import rclpy
from cartpole_interfaces.msg import Action
from cartpole_interfaces.srv import SetPosition, Stop
from rclpy.node import Node
from std_msgs.msg import Float64
from queue import Queue

"""
cartpole_teleop_key

Publishes keyboard commands as cartpole/action message
Additionally, implement a client and sends requests when appropriate

Subscribes to:

Publishes to:
    cartpole/action [cartpole_interfaces/msg/Action]             - Desired effort of the cart

Sends requests:
    cartpole/stop [cartpole_interfaces/srv/Stop]                 - Stops robot
    cartpole/set_position [cartpole_interfaces/srv/SetPosition]  - Sets absolution position of cart
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

effort_key_bindings = {'a': 1.0, 'd': -1.0}

effort_multiplier = 1.0

effort_delta_key_bindings = {'z': 1.0, 'x': -1.0}

k_queue = Queue()

exit_signal = threading.Event()


def get_terminal_settings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


class CartpoleTeleop(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('cartpole_teleop_key')

        self.effort_topic = '/slider_cart_effort_controller/command'
        self.velocity_topic = '/slider_cart_velocity_controller/command'
        self.position_topic = '/slider_cart_position_controller/command'

        self.stop_service_name = '/cartpole/stop'
        self.set_position_service_name = '/cartpole/set_position'

        self._effort_publisher = self.create_publisher(Float64,
                                                self.effort_topic,
                                                10)

        self._velocity_publisher = self.create_publisher(Float64,
                                                self.velocity_topic,
                                                10)

        self._position_publisher = self.create_publisher(Float64,
                                                self.position_topic,
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

        self.effort_multiplier = 15.0

        # Limits the value of effort
        self.max_effort = 40.0
        self.delta_effort = 1
        self.effort = 0.0

        self.velocity = 0.0

    def publish_effort(self):
        self.msg.data = self.effort

        self._effort_publisher.publish(self.msg)
        self.get_logger().info('Publishing: "%s" to topic: "%s"\n' % (self.msg.data, self.effort_topic))

    def publish_velocity(self):
        self.msg.data = self.velocity
        self._velocity_publisher.publish(self.msg)
        self.get_logger().info('Publishing: "%s" to topic: "%s"\n' % (self.msg.data, self.velocity_topic))

    def publish_position(self):
        self.msg.data = self.position
        self._position_publisher.publish(self.msg)
        self.get_logger().info('Publishing: "%s" to topic: "%s"\n' % (self.msg.data, self.position_topic))

    def limit_effort(self):
        if self.effort > self.max_effort:
            self.effort = self.max_effort
        elif self.effort < self.min_effort:
            self.effort = self.min_effort

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

def get_key(settings, key):
    while not exit_signal.is_set():
        if sys.platform == 'win32':
            key = msvcrt.getwch()
        else:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
            k_queue.put(key)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        time.sleep(0.1)

def pub_cmd(node):
    while not exit_signal.is_set():
        if not k_queue.empty():
            key = k_queue.get()
            if key:
                node.get_logger().info(key)
                if key in effort_key_bindings:
                    node.effort = node.effort_multiplier * effort_key_bindings[key]
                elif key in effort_delta_key_bindings:
                    node.effort = 0.0
                    node.effort_multiplier += node.delta_effort * effort_delta_key_bindings[key]
                    continue
                elif key == '\x03':
                    break
        
        node.publish_effort()
        time.sleep(1/30.0)


def main():
    settings = get_terminal_settings()
    node = CartpoleTeleop()

    print(msg)
    t1 = threading.Thread(target=get_key, args=(settings,k_queue))
    t2 = threading.Thread(target=pub_cmd, args=(node,))

    t1.start()
    t2.start()

    try:
        while not exit_signal.is_set():  # enable children threads to exit the main thread, too
            time.sleep(0.1)  # let it breathe a little
    except KeyboardInterrupt:  # on keyboard interrupt...
        exit_signal.set() 

    t1.join()
    t2.join()
    
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
