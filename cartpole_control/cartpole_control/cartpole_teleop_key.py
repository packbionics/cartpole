import select
import sys
import termios
import tty

import rclpy
from cartpole_interfaces.msg import Action
from cartpole_interfaces.srv import SetPosition, Stop
from rclpy.node import Node
from rclpy.qos import qos_profile_default

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
        https://github.com/ros2/teleop_twist_keyboard
        
    2. Refer to cartpole_interfaces/msg for the message formats

Services:
    1. ROS2 services overview:
        https://docs.ros.org/en/foxy/Tutorials/Services/Understanding-ROS2-Services.html

    2. ROS2 services exampl: 
        https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Service-And-Client.html 

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

class CartepoleTeleop(Node):

  def __init__(self):
    rclpy.init()
    super().__init__('cartpole_teleop_key')
                            
    self.publisher_ = self.create_publisher(
                        Action, 
                        '/cartpole/action', 
                        10)
 
    self.max_accel = 1.0

 
def main():
    node = CartepoleTeleop()
    rclpy.spin(node)
 
if __name__ == '__main__':
    main()
