import numpy as np

class Explorer:
    def __init__(self, sys, pi):
        self.sys = sys
        self.pi = pi

    def action(self, x):
        if self.sys.Px.contains(x):
            u = self.sys.Pu.randomPoint()
        else:
            u = self.pi(x)
            if not self.sys.Pu.contains(u):  # scale to lie within constraints
                aux = np.max(np.abs(self.sys.Pu.A @ u / self.sys.Pu.b))
                u = u / aux
        return u

