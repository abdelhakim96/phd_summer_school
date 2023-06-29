import cvxpy as cp
import numpy as np

class MPSF:
    def __init__(self, sys, N, P, alpha, K):
        self.sys = sys
        self.N = N
        self.P = P
        self.alpha = alpha
        self.Uc = np.zeros((sys.m, N))
        self.Xc = np.zeros((sys.n, N+1))
        self.K = K
        
        x_i = cp.Variable((sys.n, N+1))   # state
        u_i = cp.Variable((sys.m, N))     # input
        x_0 = cp.Variable((sys.n, 1))     # initial state
        u_L = cp.Variable((sys.m, 1))     # learning input
        eps = cp.Variable(1, 1)           # slack variable
        
        # Define Cost Function & Constraints
        objective_MPC = cp.Minimize(cp.sum_squares(u_i) + 1e4 * eps)
        
        constraints_MPC = [x_i[:, 0] == x_0]
        
        for i in range(N):
            # State Propagation Constraints
            constraints_MPC += [x_i[:, i+1] == sys.A @ x_i[:, i] + sys.B @ u_i[:, i]]
            
            # State & Input Constraints
            constraints_MPC += [sys.Px.A @ x_i[:, i+1] <= (1 + eps) * sys.Px.b]
            constraints_MPC += [sys.Pu.A @ u_i[:, i] <= sys.Pu.b]
            
        # Terminal constraint
        constraints_MPC += [cp.norm(cp.sqrt(P) @ x_i[:, N+1]) <= alpha]
        
        # Soft constraints / slack
        constraints_MPC += [eps >= 0]
        
        # Solve the optimization problem
        prob = cp.Problem(objective_MPC, constraints_MPC)
        self.optimizer = lambda x, uL: prob.solve(solver='SCS', verbose=True, warm_start=True, init_vals=[x_0.value, u_L.value], warm_start_value=[x, uL])

    def solve(self, x, u_L):
        # Call optimizer and check solve status
        status = self.optimizer(x, u_L)
        U = self.u_i.value
        X = self.x_i.value
        
        if status != 'optimal':
            if np.isnan(U).any():
                U = np.hstack((self.Uc[:, 1:], self.K @ self.Xc[:, -1:]))
                X = np.hstack((self.Xc[:, 1:], (self.sys.A + self.sys.B @ self.K) @ self.Xc[:, -1:]))
                print("MPSF returned NaN, overwriting with candidate solution")
        
        self.Uc = U
        self.Xc = X
        u = U[:, 0]
        
        return u, U, X

