'''
hw3p5.py

   This is a skeleton for HW3 Problem 5.  Please EDIT.

   It creates a trajectory generation node to command the joint
   movements.

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState
'''

import rclpy
import numpy as np

from math               import pi, sin, cos, acos, atan2, sqrt, fmod

from rclpy.node         import Node
from sensor_msgs.msg    import JointState


#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self):
        #### PRECOMPUTE ANY DATA YOU MIGHT NEED.
        self.position_1 = [0.675, 0.508, 0.644]
        self.position_2 = [0.675, 1.15, 5.64]
        self.position_3 = [3.81, 1.99, 0.644]
        self.position_4 = [3.81, 2.63, 5.64]

        self.velocity_1 = [float(self.position_1[0] - self.position_4[0]), float(self.position_1[1] - self.position_4[1]), float(self.position_1[2] - self.position_4[2])]
        self.velocity_2 = [float(self.position_2[0] - self.position_1[0]), float(self.position_2[1] - self.position_1[1]), float(self.position_2[2] - self.position_1[2])]
        self.velocity_3 = [float(self.position_3[0] - self.position_2[0]), float(self.position_3[1] - self.position_2[1]), float(self.position_3[2] - self.position_2[2])]
        self.velocity_4 = [float(self.position_4[0] - self.position_3[0]), float(self.position_4[1] - self.position_3[1]), float(self.position_4[2] - self.position_3[2])]
        self.velocity_0 = [float(0), float(0), float(0)]
	
    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names
        #### YOU WILL HAVE TO LOOK AT THE URDF TO DETERMINE THESE! ####
        return ['theta1', 'theta2', 'theta3']

    # Evaluate at the given time.
    def evaluate(self, t, dt):
        #### COMPUTE THE POSITION AND VELOCITY VECTORS AS A FUNCTION OF TIME.
        relative_t = t % 6 
        if relative_t >= 0 and relative_t < 1:
                return (self.position_1, self.velocity_1)
	
        elif relative_t >= 1 and relative_t < 1.5:
                return (self.position_1, self.velocity_0)
	
        elif relative_t >= 1.5 and relative_t < 2.5:
                return (self.position_2, self.velocity_2)
	
        elif relative_t >= 2.5 and relative_t < 3:
                return (self.position_2, self.velocity_0)
	
        elif relative_t >= 3 and relative_t < 4:
                return (self.position_3, self.velocity_3)
		
        elif relative_t >= 4 and relative_t < 4.5:
                return (self.position_3, self.velocity_0)
		
        elif relative_t >= 4.5 	and relative_t < 5.5:
                return (self.position_4, self.velocity_4)
	 	
        else:
                return (self.position_4, self.velocity_0)

        # Return the position and velocity as python lists.
        # return (q,qdot)


#
#   Generator Node Class
#
#   This inherits all the standard ROS node stuff, but adds an
#   update() method to be called regularly by an internal timer and a
#   shutdown method to stop the timer.
#
#   Take the node name and the update frequency as arguments.
#
class Generator(Node):
    # Initialization.
    def __init__(self, name, rate):
        # Initialize the node, naming it 'generator'
        super().__init__(name)

        # Set up the trajectory.
        self.trajectory = Trajectory()
        self.jointnames = self.trajectory.jointnames()

        # Add a publisher to send the joint commands.
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /joint_states subscriber...")
        while(not self.count_subscribers('/joint_states')):
            pass

        # Create a timer to trigger calculating/sending commands.
        self.timer     = self.create_timer(1/float(rate), self.update)
        self.dt        = self.timer.timer_period_ns * 1e-9
        self.t         = - self.dt
        self.get_logger().info("Running with dt of %f seconds (%fHz)" %
                               (self.dt, rate))

    # Shutdown
    def shutdown(self):
        # Destroy the timer, then shut down the node.
        self.timer.destroy()
        self.destroy_node()

    # Update - send a new joint command every time step.
    def update(self):
        # Grab the current time (from the ROS clock, since 1970).
        now = self.get_clock().now()

        # To avoid any time jitter enforce a constant time step in
        # integrate to get the current time.
        self.t += self.dt

        # Compute the desired joint positions and velocities for this time.
        (q, qdot) = self.trajectory.evaluate(self.t, self.dt)

        # Build up a command message and publish.
        cmdmsg = JointState()
        cmdmsg.header.stamp = now.to_msg()      # Current time for ROS
        cmdmsg.name         = self.jointnames   # List of joint names
        cmdmsg.position     = q                 # List of joint positions
        cmdmsg.velocity     = qdot              # List of joint velocities
        self.pub.publish(cmdmsg)


#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Initialize the generator node for 100Hz udpates.
    generator = Generator('generator', 100)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted.
    rclpy.spin(generator)

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
