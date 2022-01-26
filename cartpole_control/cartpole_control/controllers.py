"""
controllers.py
Controllers for cartpole swingup and balancing

Two methods: MPC and Energy-shaping
Publishes effort (force) command at specified rate
Implement handlers for state messages
"""

import math
import time

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from gekko import GEKKO as gk
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from cartpole_interfaces.srv import SetEnergyGains, SetLQRGains


"""
CartPoleSwingUpController

Base class for controllers
"""
class CartPoleSwingUpController(Node):

    def __init__(self, node_name, gravity=9.81, 
                mass_cart=1, 
                mass_pole=1, 
                length_pole=0.22, 
                k_lqr=np.array([-3.2361, 90.2449, -3.6869, 5.4608])):
        
        super().__init__(node_name)
        self.state = np.zeros((4,1))
        self.k_lqr = k_lqr
        self.gravity = gravity
        self.mass_cart = mass_cart
        self.mass_pole = mass_pole
        self.length_pole = length_pole
        
        self.k_lqr_service = self.create_service(SetLQRGains, 'set_lqr_gains', self.lqr_gains_callback)

    def lqr_gains_callback(self, request, response):
        self.k_lqr = np.array([request.k_x, 
                               request.k_theta, 
                               request.k_x_dot, 
                               request.k_theta_dot])
        self.get_logger().info('LQR gains set to: {}'.format(self.k_lqr))
        response.result = 1
        
    def unpack_parameters(self):
        return (self.gravity,
                self.mass_pole,
                self.mass_cart,
                self.length_pole,
                self.k_lqr)

    """
    state_estimate_callback
    
    Receives messages from subscribed topic and make adjustments
    In particular, PyBullet state publisher plugin flips some signs, 
        we need to account for this.
    """
    def state_estimate_callback(self, msg):
        self.state[0] = -msg.position[1]
        self.state[1] = -msg.velocity[1]
        self.state[2] = msg.position[0]
        self.state[3] = msg.velocity[0]
        
        return self.state
    
    """
    control_callback
    
    publishes effort command to topic
    """
    def control_callback(self):
        effort = Float64()
        force = float(self.get_action())
        effort.data = force
        self.publisher.publish(effort)
        
    """
    get_action
    
    Calls appropriate controller (swingup / LQR) depending on current state
    """
    def get_action(self):
        if (abs(self.theta_distance(self.state[2],math.pi)) < .3):
            return float(self.upright_lqr())
        else:
            return float(self.swingup())

    """
    theta_distance
    
    Compute the signed angle difference between theta and target
    """
    def theta_distance(self, theta, target):
        return math.atan2(math.sin(theta-target), math.cos(theta-target))

    """
    upright_lqr
    
    LQR balancing controller
    Elementwise multiplication of predetermined feedback on state errors
    See control-examples/cart-pole/generate_lqr.m for more info on gain computation
    """
    def upright_lqr(self):
        k_lqr = self.k_lqr

        x = self.state[0]
        x_dot = self.state[1]
        theta = self.state[2]
        theta_dot = self.state[3]

        theta_diff = self.theta_distance(theta,math.pi)

        X = np.array([x[0], theta_diff, x_dot[0], theta_dot[0
                                                            ]])
        f = np.dot(k_lqr,X)

        return f 

    def swingup(self):
        raise NotImplementedError


"""
CartPoleMPCController

Implements MPC for cartpole using Gekko
"""
class CartPoleMPCController(CartPoleSwingUpController):

    def __init__(self):
        
        super().__init__('cartpole_mpc_controller')

        self.loop_rate = 10.0
        self.position_topic = '/slider_cart_position_controller/command'
        self.velocity_topic = '/slider_cart_velocity_controller/command'
        self.effort_topic = '/slider_cart_effort_controller/command'

        self.publisher = self.create_publisher(Float64,
                                               self.effort_topic,
                                               10)

        self.joint_state_topic = '/joint_states'               
        self.subscriber = self.create_subscription(JointState,
                                                   self.joint_state_topic,
                                                   self.state_estimate_callback,
                                                   10)

        self.timer = self.create_timer(1.0/self.loop_rate, self.control_callback)

        self.m = gk(remote=False)

        # Number of knot points
        N = 12
        T = 1
        # Define time mesh, oversample points closer to present
        self.m.time = np.linspace(0,T,N)**2
        p = np.zeros(N)
        p[-1] = T
        final = self.m.Param(value=p)

        m1 = self.mass_cart
        m2 = self.mass_pole
        l = self.length_pole
        g = self.gravity
        
        # Specify constraints
        x_0 = [0.0,0.0,0.0,0.0]
        x_f = [0.5, math.pi, 0.0, 0.0]

        pos_lb = -1.0
        pos_ub = 1.0
        Fmx = 100.0

        self.x = self.m.Array(self.m.Var,(4))
        self.x[0].lower = pos_lb
        self.x[0].upper = pos_ub

        for i in range(4):
            self.x[i].value = x_0[i]

        self.u = self.m.MV(value=0,lb=-Fmx,ub=Fmx)
        self.u.STATUS = 1

        self.m.Equation(self.x[0].dt() == self.x[2])
        self.m.Equation(self.x[1].dt() == self.x[3])
        self.m.Equation(self.x[2].dt() == ((l*m2*self.m.sin(self.x[1])*self.x[3]**2 + g*m2*self.m.cos(self.x[1])*self.m.sin(self.x[1]) + self.u)/
                                (m1+m2*(1-self.m.cos(self.x[1])**2))))
        self.m.Equation(self.x[3].dt() == -((l*m2*self.m.cos(self.x[1])*self.m.sin(self.x[1])*self.x[3]**2+self.u*self.m.cos(self.x[1])+(m1+m2)*g*self.m.sin(self.x[1])) /
                                (l*m1+l*m2*(1-self.m.cos(self.x[1])**2))))

        #self.m.Equation((self.x[1]*final - x_f[1])**2 - 0.1 <= 0) 
        
        # Optimization Objectives
        self.m.Minimize(self.m.integral(self.u**2))
        self.m.Minimize(1e2*(self.x[0]*final-x_f[0])**2*final)
        self.m.Minimize(1e3*(self.x[1]-x_f[1])**2*final)
        #self.m.Minimize(1e5*(self.x[2]*final-x_f[2])**2*final)
        #self.m.Minimize(1e5*(self.x[3]*final-x_f[3])**2*final)

        self.m.options.IMODE = 6

        self.current_action = 0
        self.fail_count = 0
        
        self.p, = plt.plot([],[], 'r-')
        plt.title('MPC state trajectory')
        plt.xlabel('x position')
        plt.ylabel('theta')
        
    def update_line(self, new_data):
        x_data = new_data[0]
        y_data = new_data[1]

        self.p.set_xdata(x_data)
        self.p.set_ydata(y_data)
        plt.draw()
        plt.ylim((-5,5))
        plt.xlim((-2,2))
        plt.pause(0.000001)
        
    """
    swingup
    
    MPC swingup - solve for plusible trajectory
    """
    def swingup(self):
        try:
            self.x[0].value = self.state[0]
            self.x[1].value = self.state[2]
            self.x[2].value = self.state[1]
            self.x[3].value = self.state[3]
            self.m.solve()
        except:
            print('MPC fail')
            self.fail_count += 1
            if self.fail_count > 5:
                raise ValueError('MPC failed to solve for trajectory.. falling back')
            return np.array([self.current_action])

        self.fail_count = 0
        #self.update_line(np.array([self.x[0].value,self.x[1].value]))

        self.current_action = self.u.value[1]
        return self.current_action

"""
CartPoleEnergyShapingController

Energy shaping controller
k_e: Energy gain, determines how hard the cart tries to swing pole
k_x: [Position, Linear velocity] gains, higher values will regularize cart
        and prevent excessive swinging
"""
class CartPoleEnergyShapingController(CartPoleSwingUpController): 

    def __init__(self, k_e=14, k_x=[1,2]):
        
        super().__init__('cartpole_energy_shaping_controller')
        
        self.loop_rate = 40.0
        self.position_topic = '/slider_cart_position_controller/command'
        self.velocity_topic = '/slider_cart_velocity_controller/command'
        self.effort_topic = '/slider_cart_effort_controller/command'


        self.publisher = self.create_publisher(Float64,
                                               self.effort_topic,
                                               10)

        self.joint_state_topic = '/joint_states'               
        self.subscriber = self.create_subscription(JointState,
                                                   self.joint_state_topic,
                                                   self.state_estimate_callback,
                                                   10)

        self.timer = self.create_timer(1.0/self.loop_rate, self.control_callback)

        self.k_e = k_e
        self.k_x = k_x
        
        self.k_e_service = self.create_service(SetEnergyGains, 'set_energy_gains', self.energy_gains_callback)

    def energy_gains_callback(self, request, response):
        self.k_e = request.k_e
        self.k_x = [request.k_x, request.k_x_dot]
        self.get_logger().info('Energy gain set to: {}'.format(self.k_e))
        self.get_logger().info('Position gain set to: {}'.format(self.k_x[0]))
        self.get_logger().info('Velocity gain set to: {}'.format(self.k_x[1]))
        response.result = int(1)
        # TODO Fix response typeerror

    def unpack_parameters(self):
        k_e = self.k_e
        k_x = self.k_x

        return super().unpack_parameters() + (k_e, k_x)

    """
    energy
    
    Total energy of the pendulum (potential + kinetic)
    Assumes the pendulum is a point mass attached by light rod
    """
    def energy(self):
        (gravity, mass_pole,
            mass_cart,
            length_pole,
            k_lqr,
            k_e,
            k_x) = self.unpack_parameters()

        theta = self.state[2]
        theta_dot = self.state[3]

        U = -mass_pole * gravity * length_pole * math.cos(theta)
        E = 0.5 * (mass_pole * (length_pole ** 2)) * theta_dot ** 2 + U

        return E

    """
    swingup
    
    Energy-shaping swingup: Use difference of current energy and potential energy of
        upright pole to determine force command
    """
    def swingup(self):
        (gravity, mass_pole,
            mass_cart,
            length_pole,
            k_lqr, 
            k_e,
            k_x,) = self.unpack_parameters()

        Ed = mass_pole*gravity*length_pole
        Ediff = self.energy() - Ed

        x = self.state[0]
        x_dot = self.state[1]
        theta = self.state[2]
        theta_dot = self.state[3]

        c = math.cos(theta)
        s = math.sin(theta)

        acceleration = k_e * theta_dot * c * Ediff - k_x[0] * x - k_x[1]*x_dot

        f = ((mass_pole+mass_cart)*acceleration + 
                mass_pole*(-acceleration*c-gravity*s)*c - 
                mass_pole*length_pole*theta_dot**2*s)

        return -f

