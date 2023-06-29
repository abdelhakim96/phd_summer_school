import cvxpy as cp
import numpy as np

class IBSF:
    def __init__(self, sys):
        self.sys = sys
        
    def optimize(self):
        # SDP Variables
        Pi = cp.Variable((self.sys.n, self.sys.n), symmetric=True)  # Pi = P_inv
        Y = cp.Variable((self.sys.m, self.sys.n))

        # Objective: Maximize ellipsoidal volume
        objective = cp.Maximize(cp.log_det(Pi))

        # Constraints


        # Positive Definite and Lyapunov Decrease
        #Invariance
        constraints = [Pi >> 0]    #postive definte P_inv
        constraints += [Pi + Pi @ cp.transpose(self.A + self.B @self.K) >>0 ]
        constraints += [(self.A + self.B @self.K @ Y + Pi) >>0]


       #state constraints
       # for i in range(self.sys.nx):
       #     constraints += [[E*self.sys.Px.A[i].T, E*self.sys.Px.A[i].T + Y*self.sys.Px.B[i].T], [E*self.sys.Px.A[i].T + Y*self.sys.Px.B[i].T, E*self.sys.Px.b[i]]] >> 0

        # Input constraints
       # for j in range(self.sys.nu):
       #     constraints += [[self.sys.Pu.A[j]*Y.T, self.sys.Pu.A[j]*Y.T + np.eye(self.sys.n)]] >> 0

        # Solve
        prob = cp.Problem(objective, constraints)
        prob.solve(verbose=True, solver='SCS')

        self.P = E.value
        self.K = Y.value

    def filter(self, x, uL):
        u_s = np.zeros((self.sys.m, x.shape[1]))
        
        for i in range(x.shape[1]):
            x_i = x[:, i].reshape(-1, 1)
            uL_i = uL[:, i].reshape(-1, 1)
            u_i = self.K @ x_i + uL_i
            u_s[:, i] = u_i.reshape(-1)
            
            if not (self.sys.Px.A @ x_i <= self.sys.Px.b).all() or not (self.sys.Pu.A @ u_i <= self.sys.Pu.b).all():
                u_s[:, i] = uL_i.reshape(-1)
                
        return u_s

