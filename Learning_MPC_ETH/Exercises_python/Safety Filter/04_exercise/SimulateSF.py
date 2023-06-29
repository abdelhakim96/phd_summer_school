import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def SimulateSF(sys, cntr, x_init, T):
    Ns = x_init.shape[1]
    x = np.zeros((sys.n, Ns, T+1))
    u = np.zeros((sys.m, Ns, T))
    x[:, :, 0] = x_init

    for j in range(Ns):
        for i in range(T):
            u[:, j, i] = cntr(x[:, j, i])
            x[:, j, i+1] = sys.step(x[:, j, i], u[:, j, i])

    # Plot System Trajectory within Constraints
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for j in range(Ns):
        ax.plot(x[0, j, :], x[2, j, :], zs=0, color='r')

    # Plot state constraints
    vertices = sys.Px.A[:, [0, 2]]
    ax.add_collection3d(plt.Polygon(vertices, alpha=0.1))

    ax.set_xlabel('X-Position')
    ax.set_ylabel('Y-Position')
    ax.set_zlabel('Z-Position')

    plt.figure()
    plt.plot(x[1, :, :].T, x[3, :, :].T, 'r')
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlabel('X-Velocity')
    plt.ylabel('Y-Velocity')

    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(u[0, :, :].T, 'b')
    plt.plot([0, T-1], [sys.umax[0], sys.umax[0]], 'k--')
    plt.plot([0, T-1], [sys.umin[0], sys.umin[0]], 'k--')
    plt.xlim([0, T-1])
    plt.ylabel('Input 1')
    plt.xlabel('Time steps')

    plt.subplot(2, 1, 2)
    plt.plot(u[1, :, :].T, 'b')
    plt.plot([0, T-1], [sys.umax[1], sys.umax[1]], 'k--')
    plt.plot([0, T-1], [sys.umin[1], sys.umin[1]], 'k--')
    plt.xlim([0, T-1])
    plt.ylabel('Input 2')
    plt.xlabel('Time steps')

    plt.show()


