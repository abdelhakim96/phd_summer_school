import numpy as np
from scipy.spatial import HalfspaceIntersection

class LinearSystem:
    def __init__(self, A, B, Ax, bx, Au, bu):
        self.A = A
        self.B = B
        self.Px = HalfspaceIntersection(Ax, bx)
        self.Pu = HalfspaceIntersection(Au, bu)
        self.umax = self.Pu.intersections[:B.shape[1]]
        self.umin = -self.Pu.intersections[B.shape[1]:]
        self.xmax = self.Px.intersections[:A.shape[1]]
        self.xmin = -self.Px.intersections[A.shape[1]:]
        self.n = A.shape[1]
        self.m = B.shape[1]
        self.nx = bx.shape[0]
        self.nu = bu.shape[0]

    def step(self, x, u, w=None):
        if w is None:
            return self.A @ x + self.B @ u
        else:
            return self.A @ x + self.B @ u + w

