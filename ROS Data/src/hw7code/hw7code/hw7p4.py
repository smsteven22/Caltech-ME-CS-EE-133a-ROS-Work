'''hw7p4.py

   This is the skeleton code for HW7 Problem 4.  Please EDIT.

   This uses the inverse kinematics from HW 6 Problem 4, but adds a more
   complex trajectory.

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState

'''

import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Import the format for the condition number message
from std_msgs.msg import Float64

# Grab the utilities
from hw5code.GeneratorNode      import GeneratorNode
from hw5code.TransformHelpers   import *
from hw5code.TrajectoryUtils    import *

# Grab the general fkin from HW5 P5.
from hw5code.KinematicChain     import KinematicChain


#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())

        # WHAT DO YOU NEED TO DO TO INITIALIZE THE TRAJECTORY?
        self.q0 = np.radians(np.array([0, 46.5675, 0, -93.1349, 0, 0, 46.5675]).reshape((-1,1)))
        self.p0 = np.array([0.0, 0.7, 0.6]).reshape((-1,1))
        self.R0 = Reye()

        # self.pleft = np.array([0.3, 0.5, 0.15]).reshape(-1, 1)
        # self.pright = np.array([-0.3, 0.5, 0.15]).reshape(-1, 1)

        # REUSE THE PREVIOUS INVERSE KINEMATICS INITIALIZATION.
        self.q = self.q0
        self.lam = 20
        self.L = 0.4

        # Setup up the condition number publisher
        self.pub = node.create_publisher(Float64, '/condition', 10)


    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6', 'theta7']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):

        # IMPLEMENT THE TRAJECTORY.
        pd = np.array([[0, 0.95 - 0.25 * cos(t), 0.60 + 0.25 * sin(t)]]).reshape(-1, 1)
        wd = np.array([[0, 0, 0]]).reshape(-1, 1)
        vd = np.array([[0, dt * (0.95 - 0.25 * cos(t)), dt * (0.60 + 0.25 * sin(t))]]).reshape(-1, 1)
        Rd = np.identity(7)

        # REUSE THE PREVIOUS INVERSE KINEMATICS.
        qlast = self.q
        (p, R, Jv, Jw) = self.chain.fkin(qlast)

        # Part a
        J = np.vstack((Jv, Jw))
        xd = np.vstack((vd, wd))
        e = np.vstack((ep(pd, p), eR(Rd, R)))
        J_pinv = np.linalg.pinv(J)
        qdot = J_pinv @ (xd + self.lam * e)
        
        # Part b
        # J = np.vstack((Jv, Jw))
        # J_transpose = np.transpose(J)
        # gamma = 5
        # J_winv = np.linalg.inv(J_transpose @ J + (gamma ** 2) * np.identity(7)) @ J_transpose
        # xd = np.vstack((vd, wd))
        # e = np.vstack((ep(pd, p), eR(Rd, R)))
        # qdot = J_winv @ (xd + self.lam * e)

        # Part c
        # q_goal = np.array([[self.q[0, 0], self.q[1,0], self.q[2, 0], -1 * (np.pi / 2), self.q[4,0], self.q[5,0], self.q[6,0]]]).reshape(-1, 1)
        # J = np.vstack((Jv, Jw))
        # J_transpose = np.transpose(J)
        # gamma = 0.5
        # J_winv = np.linalg.inv(J_transpose @ J + (gamma ** 2) * np.identity(7)) @ J_transpose
        # xd = np.vstack((vd, wd))
        # e = np.vstack((ep(pd, p), eR(Rd, R)))
        # lam_s = 4
        # q_dot_secondary = lam_s * (q_goal - qlast)
        # qdot = J_winv @ (xd + self.lam * e) + (np.identity(7) - J_winv @ J) @ q_dot_secondary

        q = qlast + dt * qdot
        self.q = q
        # Return the position and velocity as python lists.
        return (q.flatten().tolist(), qdot.flatten().tolist())


#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Initialize the generator node for 100Hz udpates, using the above
    # Trajectory class.
    generator = GeneratorNode('generator', 100, Trajectory)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the trajectory ends.
    generator.spin()

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()