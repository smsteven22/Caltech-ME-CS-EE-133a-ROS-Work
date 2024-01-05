'''hw5p2.py

   This is skeleton code for HW5 Problem 2.  Please EDIT.

   Implement the Newton-Raphson for seven target points.

'''

import numpy as np
import matplotlib.pyplot as plt

# Grab the fkin and Jac from P1.
from hw5p1 import fkin, Jac


#
#  Utilities
#
# 360 deg wrapping:
def wraps(q):
    return np.round(q / (2*np.pi))

def unwrapped(q):
    return q - np.round(q / (2*np.pi)) * (2*np.pi)

# 3 DOF Multiplicities - return True of False!
def elbow_up(q):
    assert np.shape(q) == (3,1), "Requires 3-element column vector"
    return np.sin(q[2,0]) < 0.0

def front_side(q):
    assert np.shape(q) == (3,1), "Requires 3-element column vector"
    l1 = 1 # is this true?
    l2 = 1
    return l1 * np.cos(q[1,0]) + l2 * np.cos(q[1,0] + q[2,0]) > 0.0



#
#  Newton Raphson
#
def newton_raphson(xgoal):
    # Collect the distance to goal and change in q every step!
    xdistance = []
    qstepsize = []

    # Set the initial joint value guess.
    q = np.array([0.0, np.pi/2, -np.pi/2]).reshape(3,1)

    # IMPLEMENT THE NEWTON-RAPHSON ALGORITHM!
    # ....
    for i in range(21):
        J = Jac(q)
        J_inv = np.linalg.inv(J)

        x_i = fkin(q)
        x_diff = xgoal - x_i

        mat_product = J_inv @ x_diff

        q_i1 = q + mat_product

        x_distance = np.linalg.norm(x_diff)
        q_step_size = np.linalg.norm(q_i1 - q)

        xdistance.append(x_distance)
        qstepsize.append(q_step_size)

        q = q_i1

    print("Final Joint Value q: ")
    print(q)
    print("Elbow Up: ")
    print(elbow_up(q))
    print("Front Side: ")
    print(front_side(q))
    print("Times wrapped: ")
    print(wraps(q))
    print()

    # Create a plot of x distances to goal and q step sizes, for N steps.
    N = 20
    xdistance = xdistance[:N+1]
    qstepsize = qstepsize[:N+1]

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
    
    ax1.plot(range(len(xdistance)), xdistance)
    ax2.plot(range(len(qstepsize)), qstepsize)

    ax1.set_title(f'Convergence Data for {xgoal.T}')
    ax2.set_xlabel('Iteration')

    ax1.set_ylabel('Task Distance to Goal')
    ax1.set_ylim([0, max(xdistance)])
    ax1.set_xlim([0, N])
    ax1.set_xticks(range(N+1))
    ax1.grid()

    ax2.set_ylabel('Joint Step Size')
    ax2.set_ylim([0, max(qstepsize)])
    ax2.set_xlim([0, N])
    ax2.set_xticks(range(N+1))
    ax2.grid()

    plt.show()


#
#  Main Code
#
def main():
    # Run the test case.  Suppress infinitesimal numbers.
    np.set_printoptions(suppress=True)

    # Prcess each target (goal position).
    for xgoal in [np.array([0.5,  1.0, 0.5]).reshape((3,1)), 
                  np.array([1.0,  0.5, 0.5]).reshape((3,1)),
                  np.array([2.0,  0.5, 0.5]).reshape((3,1)),
                  np.array([0.0, -1.0, 0.5]).reshape((3,1)),
                  np.array([0.0, -0.6, 0.5]).reshape((3,1)),
                  np.array([0.5, -1.0, 0.5]).reshape((3,1)),
                  np.array([-1.0, 0.0, 0.5]).reshape((3,1))]:
        newton_raphson(xgoal)

if __name__ == "__main__":
    main()
