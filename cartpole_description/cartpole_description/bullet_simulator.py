import time
from math import pi

import pybullet as p
import pybullet_data
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped


class DemoStatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('demo_state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        degree = pi / 180.0
        loop_rate = self.create_rate(30)

        # robot state - cart and pole
        cart = 0.
        pole = 0.

        cart_inc = 0.01
        cart_inc_multiplier = 1.0
        pole_inc = degree * 2.5

        # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'rail'
        joint_state = JointState()



        physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-10)
        planeId = p.loadURDF("plane.urdf")
        startPos = [0,0,1]
        startOrientation = p.getQuaternionFromEuler([0,0,0])
        boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)
        #set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
        for i in range (10000):
            p.stepSimulation()
            time.sleep(1./240.)
        print('hello')  
        cubePos, *cubeOrn = p.getBasePositionAndOrientation(boxId)
        print(cubePos,cubeOrn)
        p.disconnect()



        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['revolute_pole', 'slider_cart']
                joint_state.position = [pole, cart]

                # update transform
                odom_trans.header.stamp = now.to_msg()

                # send the joint state
                self.joint_pub.publish(joint_state)
                self.broadcaster.sendTransform(odom_trans)
                
                # Create new robot state
                if (cart > 0.42):
                    cart_inc_multiplier = -1.0
                if (cart < -0.42):
                    cart_inc_multiplier = 1.0

                cart += cart_inc * cart_inc_multiplier

                pole += pole_inc

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass

def main():
    node = DemoStatePublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
