%% Define nonlinear state-space model for the pellet smoker
% T_f = x(1);
% T_c = x(2);
% m_p = x(3);
% u_p = u(1);
% u_f = u(2);
T_f_dot_nonlinear = @(x, u) 1/C_f * (  -k_f*(x(1)-x(2)) -k_fa*(x(1)-T_amb) +gamma*u(1)*u(2)  );
T_c_dot_nonlinear = @(x, u) 1/C_c * (   k_f*(x(1)-x(2)) -k_ca*(x(2)-T_amb)  );
m_p_dot_nonlinear = @(x, u) -u(1);
x_dot_nonlinear = @(x, u) [
    1/C_f * (  -k_f*(x(1)-x(2)) -k_fa*(x(1)-T_amb) +gamma*u(1)*u(2)  );
    1/C_c * (   k_f*(x(1)-x(2)) -k_ca*(x(2)-T_amb)  );
    -u(1);
];
y_nonlinear = @(x) [0 1 0]*x;
disp("scr_nonlinear_model: created nonlinear model")
disp("  - x_dot_nonlinear: anonymous function of inputs x and u")
disp("  - T_f_dot_nonlinear: anonymous function x_dot(1)")
disp("  - T_c_dot_nonlinear: anonymous function x_dot(2)")
disp("  - m_p_dot_nonlinear: anonymous function x_dot(3)")
%disp("  x_dot_nonlinear_vect: abstract vector representation of x dot")