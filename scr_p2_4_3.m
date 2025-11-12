%% Linearize at an equilibrium point
% Define the equilibrium point
x_e = [T_amb; T_amb; 0]; % 0 for x_3 is arbitrary, no effect
% Compute the Jacobian matrix at the equilibrium point
p2_4_3.A = jacobian(x_dot_nonlinear([x_1;x_2;x_3], [u_p; u_f]),[x_1;x_2;x_3]);
p2_4_3.B = jacobian(x_dot_nonlinear([x_1;x_2;x_3], [u_p; u_f]),[u_p;u_f]);
latex(p2_4_3.B)

