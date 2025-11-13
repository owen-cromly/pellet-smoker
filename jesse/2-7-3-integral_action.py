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
Tf0, Tc0, mp0 = 81.0, 80.0, 0.0
x_op = np.array([Tf0, Tc0, mp0])

# 1) 선형 모델
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

# 2) 2.7.2 Integral Action
A_t = np.block([
    [A,           np.zeros((3,1))],
    [-C,          np.zeros((1,1))]
])                           # (4x4)

# 두 입력(up, uf) 모두 사용 -> (4x2)
B_t = np.vstack([B, [[0.0, 0.0]]])

# 참조 r 경로 (xI_dot = r - y)
E_t = np.vstack([np.zeros((3,1)), [[1.0]]])

# 출력 (그냥 Tc를 본다)
C_t = np.hstack([C, np.zeros((1,1))])

# 3) pole 배치
desired_poles = [-0.03, -0.05, -0.08, -0.10] 

K = place_poles(A_t, B_t, desired_poles).gain_matrix  # K: (2x4)  -> u=[up, uf]^T
A_cl = A_t - B_t @ K

print("Closed-loop eigenvalues:", np.linalg.eigvals(A_cl))

Tc_sp = 100
r_delta = Tc_sp - Tc0
z0 = np.zeros(4)

def f_cl(t, z):
    return A_cl @ z + (E_t.flatten() * r_delta)

t = np.linspace(0.0, 200.0, 401)
sol = solve_ivp(f_cl, (t[0], t[-1]), z0, t_eval=t, rtol=1e-7, atol=1e-9)

# 출력 Tc(t)
Tc_dev = (C_t @ sol.y).ravel() 
Tc_cl = Tc0 + Tc_dev

print("t.shape:", t.shape, "  Tc_cl.shape:", Tc_cl.shape)

plt.figure()
plt.plot(t, Tc_cl, label='Tc')
plt.xlabel('Time (s)')
plt.ylabel('Chamber temperature Tc (°C)')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
