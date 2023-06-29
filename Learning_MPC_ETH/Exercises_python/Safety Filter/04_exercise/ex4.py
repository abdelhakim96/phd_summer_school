import numpy as np
from LinearSystem import LinearSystem
from IBSF import IBSF

# Create the linear system instance
A = np.array([[0.5, 0.2], [0.1, 0.7]])
B = np.array([[1], [0]])
Ax = np.array([[1, 0], [0, 1], [-1, 0], [0, -1]])
bx = np.array([1, 1, 1, 1])
Au = np.array([[1], [-1]])
bu = np.array([1, 1])
sys = LinearSystem(A, B, Ax, bx, Au, bu)

# Create the IBSF instance
ibsf = IBSF(sys)

# Optimize the controller parameters
ibsf.optimize()

# Example usage of the IBSF class
x = np.array([[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]])  # Current state
uL = np.array([[0.1, 0.2, 0.3]])  # Learning input
u_s = ibsf.filter(x, uL)  # Compute filtered inputs

print("Filtered inputs:")
print(u_s)


