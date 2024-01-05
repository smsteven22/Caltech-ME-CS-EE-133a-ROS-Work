'''hw7p2.py

   This is the skeleton code for HW7 Problem 2.  Please EDIT.

   This uses the inverse kinematics from HW6 Problem 4, but adds a more
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
        self.q0 = np.radians(np.array([0, 90, -90, 0, 0, 0]).reshape((-1,1)))
        self.p0 = np.array([0.0, 0.55, 1.0]).reshape((-1,1))
        self.R0 = Reye()

        self.pleft = np.array([0.3, 0.5, 0.15]).reshape(-1, 1)
        self.pright = np.array([-0.3, 0.5, 0.15]).reshape(-1, 1)

        # REUSE THE PREVIOUS INVERSE KINEMATICS INITIALIZATION.
        self.q = self.q0
        self.lam = 20
        self.L = 0.4

        # Setup up the condition number publisher
        self.pub = node.create_publisher(Float64, '/condition', 10)


    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):

        # IMPLEMENT THE TRAJECTORY.
        if t < 3.0:
            (pd, vd) = goto(t, 3.0, self.p0, self.pright)
            Rd = Reye()
            wd = np.array([0, 0, 0]).reshape(-1, 1)
        else:
            t1 = (t - 3) % 5
            if t1 < 1.25:
                # Right to middle
                (pd, vd) = goto(t1, 1.25, self.pright, self.p0)
                (theta, thetadot) = goto(t1, 1.25, 0, np.radians(-90)) 
                Rd = Roty(theta)
                wd = ey() * thetadot
            elif t1 >= 1.25 and t1 < 2.5:
                # Middle to left
                (pd, vd) = goto(t1 - 1.25, 1.25, self.p0, self.pleft)
                (theta, thetadot) = goto(t1 - 1.25, 1.25, 0, np.radians(90))
                Rd = (Roty(np.radians(-90))) @ (Rotz(theta))
                wd = ez() * thetadot
            elif t1 >= 2.5 and t1 < 3.75:
                # Left to middle
                (pd, vd) = goto(t1 - 2.5, 1.25, self.pleft, self.p0)
                (theta, thetadot) = goto(t1 - 2.5, 1.25, 0, np.radians(-90))
                Rd = (Roty(np.radians(-90))) @ (Rotz(np.radians(90))) @ (Rotz(theta))
                wd = ez() * thetadot
            else:
                # Middle to right
                (pd, vd) = goto(t1 - 3.75, 1.25, self.p0, self.pright)
                (theta, thetadot) = goto(t1 - 3.75, 1.25, 0, np.radians(90))
                Rd = (Roty(np.radians(-90))) @ Roty(theta)
                wd = ey() * thetadot


        # REUSE THE PREVIOUS INVERSE KINEMATICS.
        qlast = self.q
        (p, R, Jv, Jw) = self.chain.fkin(qlast)

        J = np.vstack((Jv, Jw))
        v = np.vstack((vd, wd))
        e = np.vstack((ep(pd, p), eR(Rd, R)))
        J_pinv = np.linalg.pinv(J)

        J_bar_diag = np.diag([1/self.L, 1/self.L, 1/self.L, 1, 1, 1])
        J_bar = J_bar_diag @ J
        condition = np.linalg.cond(J_bar)

        # Publish the condition number.
        msg = Float64()
        msg.data = condition
        self.pub.publish(msg)

        qdot = J_pinv @ (v + self.lam * e)
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

