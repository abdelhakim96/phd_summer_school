import casadi as ca

class NominalMPC:
    def __init__(self, sys, N, Q, R, P, alpha):
        # Class constructor setting up optimization problem of MPC
        self.sys = sys
        self.N = N
        self.Q = Q
        self.R = R
        self.P = P
        self.alpha = alpha

        x_i = ca.MX.sym('x', sys.n, N+1)  # state
        u_i = ca.MX.sym('u', sys.m, N)  # input
        x_0 = ca.MX.sym('x0', sys.n)  # initial state

        # Define Cost Function & Constraints
        objective_MPC = 0

        # Initial State Constraint
        constraints_MPC = [x_i[:, 0] - x_0]

        for i in range(N):
            # Stage cost
            objective_MPC += ca.mtimes([x_i[:, i].T, Q, x_i[:, i]]) + ca.mtimes([u_i[:, i].T, R, u_i[:, i]])

            # State Propagation Constraints
            constraints_MPC += [x_i[:, i+1] - (sys.A @ x_i[:, i] + sys.B @ u_i[:, i])]

            # State & Input Constraints
            constraints_MPC += [sys.x_min - x_i[:, i], x_i[:, i] - sys.x_max]
            constraints_MPC += [sys.u_min - u_i[:, i], u_i[:, i] - sys.u_max]

        # Terminal Cost
        objective_MPC += ca.mtimes([x_i[:, N].T, P, x_i[:, N]])

        # Terminal constraint
        constraints_MPC += [ca.mtimes([x_i[:, N].T, P, x_i[:, N]]) - alpha]

        # Set up the optimization problem
        nlp = {'x': ca.vertcat(x_i, u_i), 'f': objective_MPC, 'g': ca.vertcat(*constraints_MPC)}
        opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.tol': 1e-4, 'ipopt.warm_start_init_point': 'yes'}
        self.optimizer = ca.nlpsol('solver', 'ipopt', nlp, opts)

    def solve(self, x):
        # Call optimizer and check solve status
        x_0 = x

        res = self.optimizer(x0=ca.vertcat(x_0, ca.zeros(self.sys.m * self.N)))  # Solve the optimization problem

        if res['success']:
            u = res['x'][self.sys.n * (self.N+1):].full()[:, 0]
            U = res['x'][self.sys.n * (self.N+1):].full()
            X = res['x'][:self.sys.n * (self.N+1)].full().reshape((self.sys.n, self.N+1))
        else:
            u = ca.zeros(self.sys.m).full()[:, 0]
            U = ca.zeros((self.sys.m, self.N)).full()
            X = ca.zeros((self.sys.n, self.N+1)).full()

        return u, U, X
