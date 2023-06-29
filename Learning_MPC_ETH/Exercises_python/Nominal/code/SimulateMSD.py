import numpy as np
import matplotlib.pyplot as plt

def SimulateMSD(sys, cntr, x_init, T, w=None):
    if w is None:
        w = np.zeros((x_init.shape[0], T))

    x = np.zeros((sys.n, T+1))
    u = np.zeros((sys.m, T))
    x[:, 0] = x_init

    for i in range(T):
        u[:, i] = cntr(x[:, i])
        x[:, i+1] = sys.step(x[:, i], u[:, i], w[:, i])

    # Plot System Trajectory within Constraints
    fig1, ax1 = plt.subplots()
    ax1.plot(x[0, :], x[1, :])
    ax1.plot(sys.Px[:, 0], sys.Px[:, 1], alpha=0.1)
    ax1.set_xlabel('Position')
    ax1.set_ylabel('Velocity')

    fig2, axs2 = plt.subplots(3, 1, sharex=True)
    axs2[0].plot(x[0, :T])
    axs2[0].plot([1, T], [sys.xmax[0], sys.xmax[0]], 'k--')
    axs2[0].plot([1, T], [sys.xmin[0], sys.xmin[0]], 'k--')
    axs2[0].set_xlim([1, T])
    axs2[0].set_ylabel('Position')

    axs2[1].plot(x[1, :])
    axs2[1].plot([1, T], [sys.xmax[1], sys.xmax[1]], 'k--')
    axs2[1].plot([1, T], [sys.xmin[1], sys.xmin[1]], 'k--')
    axs2[1].set_xlim([1, T])
    axs2[1].set_ylabel('Velocity')

    axs2[2].plot(u[0, :])
    axs2[2].plot([1, T], [sys.umax[0], sys.umax[0]], 'k--')
    axs2[2].plot([1, T], [sys.umin[0], sys.umin[0]], 'k--')
    axs2[2].set_xlim([1, T])
    axs2[2].set_ylabel('Input')
    axs2[2].set_xlabel('Time steps')

    plt.show()

    return x, u

