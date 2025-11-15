function [x_hat_dot, integral_hat_dot] = integral_observer(x_hat, integral_hat, lin, setpoint, y, u)
% INTEGRAL OBSERVER 
%   Detailed explanation goes here
arguments (Input)
    x_hat % the present value of x_hat (solved by ode45)
    integral_hat % the integral of the predicted x_hat v. setpoint error
    lin % the linearized system, as a struct
    setpoint % the setpoint of the system (used for integrating setpoint error)
    y % T_C
    u % the present value of input
end

arguments (Output)
    x_hat_dot % estimated state
    integral_hat_dot % the predicted x_hat v. setpoint error, to be integrated
end

A2 = lin.A(1:2,1:2);
L = [place(A2, [0;1], [-3.1 -2.9])'; 0]; % arbitrary poles

%C = [0 1 0];
%[C; C*A; C*A*A]

x_hat_dot = lin.A*x_hat+lin.B*(u)+L*(y-x_hat(2));
integral_hat_dot = y - setpoint(2);
integral_hat_dot = integral_hat_dot - integral_hat*0.01;
%x_hat_d(2) = y;

end