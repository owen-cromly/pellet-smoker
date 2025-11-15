import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

Cf   = 500.0
Cc   = 2500.0
kf   = 150.0
kfa  = 20.0
kca  = 25.0
gamma = 1000.0
Tamb  = 25.0

up0, uf0 = 0.4, 0.6
Tf0, Tc0, mp0 = 31, 30, 0.0
x_op = np.array([Tf0, Tc0, mp0])

# 1) linear model
A = np.array([
    [-(kf + kfa)/Cf,  kf/Cf,          0.0],
    [ kf/Cc,         -(kf + kca)/Cc,  0.0],
    [ 0.0,            0.0,            0.0]
])

B = np.array([
    [(gamma/Cf)*uf0, (gamma/Cf)*up0],
    [0.0,            0.0],
    [-1.0,           0.0]
])

C = np.array([[0.0, 1.0, 0.0]])

t = np.linspace(0, 350, 1400)

x0 = np.array([Tf0, Tc0, mp0])
dx0 = np.zeros(3) 
def f_lin(t, dx): 
    du = np.array([1.0, 0.0]) 
    return A @ dx + B @ du

sol_lin = solve_ivp(f_lin, (t[0], t[-1]), dx0, t_eval=t)
Tc_lin = 30.0 + (C @ sol_lin.y).ravel()

def f_nl(t, x):
    Tf, Tc, mp = x
    up = up0 + 1.0 
    uf = uf0
    qcomb = gamma * up * uf
    dTf = (-kf*(Tf-Tc) - kfa*(Tf-Tamb) + qcomb)/Cf
    dTc = ( kf*(Tf-Tc) - kca*(Tc-Tamb))/Cc
    dmp = -up
    return np.array([dTf, dTc, dmp])

sol_nl = solve_ivp(f_nl, (t[0], t[-1]), x0, t_eval=t)
Tc_nl = sol_nl.y[1]

plt.plot(t, Tc_lin, label='Linearized model')
plt.plot(t, Tc_nl,  label='Nonlinear model', linestyle='--')
plt.xlabel('Time'); 
plt.ylabel('Tc (°C)'); 
plt.grid(True); 
plt.legend(); 
plt.show()
