import numpy as np
from scipy.integrate import solve_ivp
from scipy.signal import place_poles
import matplotlib.pyplot as plt

# 0) 파라미터
Cf   = 500.0
Cc   = 2500.0
kf   = 150.0
kfa  = 20.0
kca  = 25.0
gamma = 1000.0
Tamb  = 25.0

up0, uf0 = 0.4, 0.6
Tf0, Tc0, mp0 = 70, 70, 0.0
x_op = np.array([Tf0, Tc0, mp0])

# 1) 선형 모델
A = np.array([
    [-(kf + kfa)/Cf,  kf/Cf, 0.0],
    [ kf/Cc, -(kf + kca)/Cc, 0.0],
    [ 0.0, 0.0, 0.0]
])

B = np.array([
    [(gamma/Cf)*uf0, (gamma/Cf)*up0],
    [0.0, 0.0],
    [-1.0, 0.0]
])

C = np.array([[0.0, 1.0, 0.0]])

# 2) 2.7.2 Integral Action
A_t = np.block([
    [A, np.zeros((3,1))],
    [-C, np.zeros((1,1))]
])

B_t = np.vstack([B, [[0.0, 0.0]]])
E_t = np.vstack([np.zeros((3,1)), [[1.0]]])
C_t = np.hstack([C, np.zeros((1,1))])

# 3) pole 배치
desired_poles = [-0.03, -0.05, -0.08, -0.10] 

K = place_poles(A_t, B_t, desired_poles).gain_matrix 
A_cl = A_t - B_t @ K

Tc_sp = 100
r_delta = Tc_sp - Tc0
z0 = np.zeros(4)

def f_cl(t, z):
    return A_cl @ z + (E_t.flatten() * r_delta)

t = np.linspace(0.0, 200.0, 401)
sol = solve_ivp(f_cl, (t[0], t[-1]), z0, t_eval=t, rtol=1e-7, atol=1e-9)

Tc_dev = (C_t @ sol.y).ravel() 
Tc_cl = Tc0 + Tc_dev

print("t.shape:", t.shape, "  Tc_cl.shape:", Tc_cl.shape)

plt.figure()
plt.plot(t, Tc_cl, label='Tc')
plt.xlabel('Time (s)')
plt.title('Integral Gain')
plt.ylabel('Chamber temperature Tc (°C)')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# 2.7.4 Luenberger observer for [Tf, Tc]

A11 = A[0:2, 0:2] 
C1  = np.array([[0.0, 1.0]])  # y = Tc = [0 1] x1

# Observability check
O1 = np.vstack([C1,
                C1 @ A11])
print("rank(O1) =", np.linalg.matrix_rank(O1)) 

# observer poles
observer_poles = [-0.12, -0.18]

L1 = place_poles(A11.T, C1.T, observer_poles).gain_matrix.T
print("Observer L1 =\n", L1)

L = np.vstack([L1,
               [[0.0],
                [0.0]]])
print("L_full =\n", L)

Tc_sp = 100.0
r_delta = Tc_sp - Tc0

def f_cl_with_observer(t, x_all):
    z = x_all[0:4] # augmented state
    z_hat = x_all[4:8] # observer state 
    y = (C_t @ z)[0]
    y_hat = (C_t @ z_hat)[0]

    u = (-K @ z_hat).flatten() 

    z_dot = A_t @ z + B_t @ u + (E_t.flatten() * r_delta)

    z_hat_dot = A_t @ z_hat + B_t @ u \
                + (E_t.flatten() * r_delta) \
                + (L * (y - y_hat)).flatten()

    return np.hstack([z_dot, z_hat_dot])

z0_true = np.zeros(4)

z0_hat  = np.array([-10, 0.0, 0.0, 0.0])

x0_all  = np.hstack([z0_true, z0_hat])

t = np.linspace(0.0, 200.0, 401)
sol = solve_ivp(f_cl_with_observer, (t[0], t[-1]), x0_all, t_eval=t, rtol=1e-7, atol=1e-9)

z     = sol.y[0:4, :]
z_hat = sol.y[4:8, :]

Tc_true = Tc0 + (C_t @ z).ravel()
Tc_hat  = Tc0 + (C_t @ z_hat).ravel()

plt.figure()
plt.plot(t, Tc_true, label='True Tc')
plt.plot(t, Tc_hat, '--', label='Estimated Tc (observer)')
plt.axhline(Tc_sp, linestyle=':', color='k', label='Setpoint')
plt.xlabel('Time (s)')
plt.ylabel('Tc (°C)')
plt.grid(True)
plt.legend()
plt.title('Closed-loop Tc with Luenberger observer')
plt.tight_layout()
plt.show()
