'''hw6p3.py

   This is the skeleton code for HW6 Problem 3.  Please EDIT.

   This creates a purely rotational movement.

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState

'''

import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

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

        # Initialize the current joint position to the starting
        # position and set the desired orientation to match.
        self.q = np.zeros((3,1))
        (_, self.Rd, _, _) = self.chain.fkin(self.q)

        # Pick the convergence bandwidth.
        self.lam = 20

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['pan', 'tilt', 'roll']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        # Choose the alpha/beta angles based on the phase.
        if t <= 2.0:
            # Part A (t<=2):
            (alpha, alphadot) = goto(t, 2.0, 0, -1.571)
            (beta,  betadot)  = (0.0, 0.0)
        else:
            # To test part A only, you can return None and the node will exit.
            # return None

            # Part B (t>2):
            (alpha, alphadot) = goto(2.0, 2.0, 0, np.radians(-90))
            (beta,  betadot)  = (t - 3 + exp(2 - t), 1 - exp(2 - t))

        # Compute the desired rotation and angular velocity.
        # WHAT ARE Rd and wd?
        Rd = Roty(alpha)
        wd = pe(ey(), alphadot)

        if (t > 2):
            e_xy = exyz(1.0, 1.0, 0.0)
            Rd = Rote(e_xy, beta) @ Roty(-1.571)
            wd = pe(e_xy, betadot)

        # Grab the stored information from last cycle.
        # WHAT DATA FROM LAST CYCLE DO WE NEED?
        qlast = self.q

        # Compute the old forward kinematics.
        (p, R, Jv, Jw) = self.chain.fkin(qlast)

        # Compute the inverse kinematics
        # INVERSE KINEMATICS FOR ROTATION
        Jw_inv = np.linalg.inv(Jw)
        e_R = eR(Rd, R)
        qdot = Jw_inv @ (wd + self.lam * e_R)
        # Integrate the joint position.
        q = qlast + dt * qdot

        # Save the data needed next cycle.
        # STORE
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
