
#ex.1 plot state constraints
import NominalMPC
import SimulateMSD
import timeit
import control
import casadi as ca
import matplotlib.pyplot as plt
import numpy as np
import numpy.linalg as linalg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


#system params
k=0.3
d=0.2



# Compute linearized system
A = np.array([[0, 1], [-k, -d]])
B = np.array([[0], [1]])
n = 2
m = 2

# LQR controller
Q = 1 * np.eye(2)
R = np.eye(1)

# compute K
K, S, E = control.lqr(A, B, Q, R)

x_ref=np.array([0.0, 0.0])
#u = K(x_ref - x)

[x,u] = SimulateMSD(sys, cntr, x_init, T, w=None)
