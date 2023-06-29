import casadi as ca

class LinearSystem:
    def __init__(self, A, B, Ax, bx, Au, bu):
        self.A = A
        self.B = B
        self.Px = ca.Polyhedron(Ax, bx)
        self.Pu = ca.Polyhedron(Au, bu)
        self.n = A.shape[1]
        self.m = B.shape[1]
        self.nx = bx.shape[0]
        self.nu = bu.shape[0]

        # Compute bounding boxes
        aux = self.Pu.outer_approx()
        self.umax = aux.b[:self.m]
        self.umin = -aux.b[self.m:]

        aux = self.Px.outer_approx()
        self.xmax = aux.b[:self.n]
        self.xmin = -aux.b[self.n:]

    def step(self, x, u, w=None):
        if w is None:  # Autonomous
            xp = ca.mtimes(self.A, x)
        elif w.shape[0] == self.n:  # Controlled with disturbance
            xp = ca.mtimes(self.A, x) + ca.mtimes(self.B, u) + w
        else:  # Controlled without disturbance
            xp = ca.mtimes(self.A, x) + ca.mtimes(self.B, u)

        return xp

