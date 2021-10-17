import rclpy
from cartpole_interfaces.msg import Action, State
from rclpy.node import Node
import numpy as np
import Math

"""
energy_shaping_controller

Subscribes to:
    cartpole/state [cartpole_interfaces/msg/State]   - State vector of robot

Publishes to:
    cartpole/action [cartpole_interfaces/msg/Action] - Desired acceleration of the cart
"""

"""
TODO: Implement energy shaping control + LQR balancing

Useful links:

Publisher / Subscriber:
    1. ROS2 publisher/subscriber demo: 
        https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html

    2. Example state estimation & control (see "Creating an Estimator and a Controller Node): 
        https://automaticaddison.com/how-to-simulate-a-robot-using-gazebo-and-ros-2/

"""
 
class EnergyShapingController(Node):

    # Robot physical parameters
    # TODO: May be use ROS2 params for these
    gravity = 9.81
    mass_pole = 1
    mass_cart = 1
    length_pole = 1

    # Control parameters
    # TODO: May be use ROS2 params for these
    k_e = 2
    k_x = [5,5]
    k_lqr = np.array([-4.47,70,-6,10.45])

    def __init__(self):
        rclpy.init()
        super().__init__('energy_shaping_controller')

        self.subscription = self.create_subscription(
                                State,
                                '/cartpole/state',
                                self.state_estimate_callback,
                                10)
                                
        self.publisher = self.create_publisher(
                                Action, 
                                '/cartpole/action', 
                                10)

        self.state = np.zeros((4,1))

    @classmethod
    def unpack_parameters(cls):
        gravity = EnergyShapingController.gravity
        mass_pole = EnergyShapingController.mass_pole
        mass_cart = EnergyShapingController.mass_cart
        length_pole = EnergyShapingController.length_pole
        k_e = EnergyShapingController.k_e
        k_x = EnergyShapingController.k_x
        k_lqr = EnergyShapingController.k_lqr

        return (gravity,
                mass_pole,
                mass_cart,
                length_pole,
                k_e,
                k_x,
                k_lqr)

    def state_estimate_callback(self, msg):
        self.state[0] = msg.x
        self.state[1] = msg.x_dot
        self.state[2] = msg.theta
        self.state[3] = msg.theta_dot

    def get_action(self):
        action = Action()
        action.x_dot2 = 0.0

        if (abs(self.theta_distance(self.state[2],Math.pi)) < .3):
            action.x_dot2 = self.upright_lqr()
        else:
            action.x_dot2 = self.swingup()

        self.publisher.publish(action) 

    def theta_distance(self, theta, target):
        """
        Computes X-X*, where X* is the target angle
        Wraps angle every 2pi to get measured difference

        Keyword arguments:
        theta   current angle (-inf,inf)
        target  target angle
        """
        diff = theta - target

        phi = abs(diff) % 2*Math.pi

        if (phi > Math.pi):
            distance = 2*Math.pi - phi
        else:
            distance = phi

        if (((diff >= 0) and (diff >= -Math.pi)) or ((diff >= Math.pi) and (diff <= 2*Math.pi)) or (diff)):
            distance = 1*distance
        else:
            distance = -1*distance

        return distance

    def energy(self):
        """
        Total energy of pendulum
        Assumes the pendulum is a point mass attached by light rod
        """
        (gravity, mass_pole,
            mass_cart,
            length_pole,
            k_e,
            k_x,
            k_lqr) = EnergyShapingController.unpack_parameters()

        theta = self.state[2]
        theta_dot = self.state[3]

        U = -mass_pole * gravity * length_pole * Math.cos(theta)
        E = 0.5 * (mass_pole * (length_pole ** 2)) * theta_dot ** 2 + U

        return E

    def swingup(self):
        """
        Cart-pole energy shaping control
        """

        (gravity, mass_pole,
            mass_cart,
            length_pole,
            k_e,
            k_x,
            k_lqr) = EnergyShapingController.unpack_parameters()

        Ed = mass_pole*gravity*length_pole
        Ediff = self.energy() - Ed

        x = self.state[0]
        x_dot = self.state[1]
        theta = self.state[2]
        theta_dot = self.state[3]

        c = Math.cos(theta)
        #s = Math.sin(theta)
        #t = Math.tan(theta)
        
        desired_acceleration = k_e * theta_dot * c * Ediff - k_x[0] * x - k_x[1]*x_dot

        #f = ((mass_pole+mass_cart)*acceleration + 
        #        mass_pole*(-acceleration*c-g*s)*c - 
        #        mass_pole*length*theta_dot**2*s)

        return desired_acceleration

    def upright_lqr(self):
        """
        LQR controller
        """
        
        (gravity, mass_pole,
            mass_cart,
            length_pole,
            k_e,
            k_x,
            k_lqr) = EnergyShapingController.unpack_parameters()

        x = self.state[0]
        x_dot = self.state[1]
        theta = self.state[2]
        theta_dot = self.state[3]

        c = Math.cos(theta)
        s = Math.sin(theta)

        theta_diff = self.theta_distance(theta,Math.pi)
        X = np.array([x, theta_diff, x_dot, theta_dot])
        f = np.dot(k_lqr,X)
        desired_acceleration = (f + mass_pole * length_pole * Math.sqrt(theta_dot) * s + gravity * s * c * mass_pole) / \
                                (mass_pole + mass_cart - mass_pole * Math.sqrt(c))

        return -1 * desired_acceleration 
 
def main():
    node = EnergyShapingController()
    rclpy.spin(node)
 
if __name__ == '__main__':
    main()
