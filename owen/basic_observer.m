function [x_hat_dot, x_I_dot] = basic_observer(x_hat, lin, y, u)
%BASIC OBSERVER 
%   Detailed explanation goes here
arguments (Input)
    x_hat % the present value of x_hat (solved by ode45)
    lin % the linearized system, as a struct
    y % T_C
    u % the present value of input
end

arguments (Output)
    x_hat_dot % estimated state
    x_I_dot % the predicted x_hat v. setpoint error, to be integrated
end

A2 = lin.A(1:2,1:2);
L = [place(A2, [0;1], [-3.1 -2.9])'; 0]; % arbitrary poles

x_hat_dot = lin.A*x_hat+lin.B*(u)+L*(y-x_hat(2));
x_I_dot = 0;

end