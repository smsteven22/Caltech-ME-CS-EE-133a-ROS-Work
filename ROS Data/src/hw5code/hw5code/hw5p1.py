'''hw5p1.py

   This is skeleton code for HW5 Problem 1.  Please EDIT.

   This should simply use NumPy to implement the known forward
   kinematics and Jacobian functions for the 3 DOF robot.

'''

import numpy as np


#
#  EXAMPLE CODE
#
#  TO SEE HOW TO USE NUMPY, LOOK AT NumPyExamples.py!
#
# REMOVE/COMMENT THIS OUT TO AVOID ALL THE EXTRA PRINTS
# import hw5code.NumPyExamples

def example():
    print("EXAMPLE CODE:")
    
    # RANDOM SAMPLE CODE 3x1 column vec, 3x3 matrix:
    q    = np.array([0.1, 0.3, -0.5]).reshape(-1,1)
    xdot = np.array([3,   6,    2  ]).reshape(-1,1)
    J    = Jac(q)
    qdot = np.linalg.inv(J) @ xdot
    print("qdot:\n", qdot)

    print("sin(q):\n",      np.sin(q))
    print("sin(q[0,0]):\n", np.sin(q[0,0]))
    print("sin(q[1,0]):\n", np.sin(q[1,0]))
    print("sin(q[2,0]):\n", np.sin(q[2,0]))
    # Remember every vector is a 2D object and needs 2 indices!


#
#  Forward Kinematics
#
def fkin(q):
    theta_pan = q[0,0]
    theta_1 = q[1,0]
    theta_2 = q[2,0]

    row_1 = (-1 * np.sin(theta_pan)) * (np.cos(theta_1) + np.cos(theta_1 + theta_2))
    row_2 = np.cos(theta_pan) * (np.cos(theta_1) + np.cos(theta_1 + theta_2))
    row_3 = np.sin(theta_1) + np.sin(theta_1 + theta_2)

    x = np.array([row_1, row_2, row_3]).reshape(-1, 1)

    # Return the tip position as a numpy 3x1 column vector.
    return x


#
#  Jacobian
#
def Jac(q):
    theta_pan = q[0,0]
    theta_1 = q[1,0]
    theta_2 = q[2,0]

    row1_col1 = (-1 * np.cos(theta_pan)) * (np.cos(theta_1) + np.cos(theta_1 + theta_2))
    row2_col1 = (-1 * np.sin(theta_pan)) * (np.cos(theta_1) + np.cos(theta_1 + theta_2))
    row3_col1 = 0

    row1_col2 = np.sin(theta_pan) * (np.sin(theta_1) + np.sin(theta_1 + theta_2))
    row2_col2 = (-1 * np.cos(theta_pan)) * (np.sin(theta_1) + np.sin(theta_1 + theta_2))
    row3_col2 = np.cos(theta_1) + np.cos(theta_1 + theta_2)

    row1_col3 = np.sin(theta_pan) * np.sin(theta_1 + theta_2)
    row2_col3 = (-1 * np.cos(theta_pan)) * np.sin(theta_1 + theta_2)
    row3_col3 = np.cos(theta_1 + theta_2)

    J = np.array([[row1_col1, row1_col2, row1_col3],
                  [row2_col1, row2_col2, row2_col3],
                  [row3_col1, row3_col2, row3_col3]])

    # Return the Jacobian as a numpy 3x3 matrix.
    return J


#
#  Main Code
#
def main():
    # Run the test case.  Suppress infinitesimal numbers.
    np.set_printoptions(suppress=True)

    # Run the example code.  FEEL FREE TO REMOVE.
    # example()

    # First (given) test case with following joint coordinates.  Make
    # q a column vector by writing it as a list of lists.
    print("TEST CASE #1:")
    q = np.array([[np.radians(20)],
                  [np.radians(40)],
                  [np.radians(-30)]])
    print('q:\n',       q)
    print('fkin(q):\n', fkin(q))
    print('Jac(q):\n',  Jac(q))

    # Second test case with following joint coordinates.  Make
    # q a column vector by explicitly reshaping.
    print("TEST CASE #2")
    q = np.radians(np.array([30, 30, 60])).reshape(3,1)
    print('q:\n',       q)
    print('fkin(q):\n', fkin(q))
    print('Jac(q):\n',  Jac(q))

if __name__ == "__main__":
    main()
