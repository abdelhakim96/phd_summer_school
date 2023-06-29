class RobustMPC:
    def __init__(self, sys, N, Q, R, P, alpha, F, K):
        self.sys = sys
        self.N = N
        self.Q = Q
        self.R = R
        self.P = P
        self.alpha = alpha
        self.F = F
        self.K = K
        
        x_i = casadi.MX.sym('x', sys.n, N+1)  # state
        u_i = casadi.MX.sym('u', sys.m, N)  # input
        x_0 = casadi.MX.sym('x0', sys.n, 1)  # initial state

        # --------- Start Modifying Code Here -----------
#         # Define Cost Function & Constraints
#         objective_MPC = 0
# 
#         # Initial State Constraint
#         constraints_MPC = [x_i[:, 1] == x_0]
# 
#         for i in range(N):
#             # Stage cost
#             objective_MPC = objective_MPC + TODO
# 
#             # State Propagation Constraints
#             constraints_MPC = casadi.vertcat(constraints_MPC, TODO)
# 
#             # State & Input Constraints
#             # Use MPT Toolbox to compute tightened constraints
#             X_t[i+1] = sys.Px - TODO
#             U_t[i] = sys.Pu - TODO
#             constraints_MPC = casadi.vertcat(constraints_MPC, TODO)
#             constraints_MPC = casadi.vertcat(constraints_MPC, TODO)
# 
#         # Terminal Cost
#         objective_MPC = objective_MPC + TODO
# 
#         # Terminal constraint
#         constraints_MPC = casadi.vertcat(constraints_MPC, TODO)
        # --------- Stop Modifying Code Here -----------
        
        # Create optimizer
        constraints = [x_0]
        constraints += x_i[:, 1:] - self.sys.step(x_i[:, :-1], u_i, casadi.MX.zeros(sys.n, N))
        constraints += self.sys.Px - TODO
        constraints += self.sys.Pu - TODO
        objective = TODO
        self.optimizer = casadi.Opti()
        self.optimizer.set_initial(x_i, np.zeros((sys.n, N+1)))
        self.optimizer.set_initial(u_i, np.zeros((sys.m, N)))
        self.optimizer.minimize(objective)
        self.optimizer.subject_to(constraints)
        
    def solve(self, x):
        # Call optimizer and check solve status
        self.optimizer.set_value(self.optimizer.value_input(), x)
        try:
            self.optimizer.solve()
            u = self.optimizer.value(u_i)[:, 0]
            X = self.optimizer.value(x_i)
            U = casadi.horzcat(u.reshape(self.sys.m, 1), self.optimizer.value(u_i)[:, 1:])
        except:
            u = np.zeros(self.sys.m)
            U = np.zeros((self.sys.m, self.N))
            X = np.zeros((self.sys.n, self.N+1))
            print("Error: Failed to solve optimization problem")

        return u, U, X

