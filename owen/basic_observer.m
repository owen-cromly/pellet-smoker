function x_hat_d = basic_observer(y,x_hat,u,lin)
%BASIC OBSERVER 
%   Detailed explanation goes here
arguments (Input)
    y % T_C
    x_hat % the present value of x_hat (solved by ode45)
    u % the present value of input
    lin % the linearized system, as a struct
end

arguments (Output)
    x_hat_d % estimated state
end

A2 = lin.A(1:2,1:2);
L = [place(A2, [0;1], [-3.1 -2.9])'; 0]; % arbitrary poles

%C = [0 1 0];
%[C; C*A; C*A*A]

x_hat_d = lin.A*x_hat+lin.B*u+L*(y-x_hat(2));
%x_hat_d(2) = y;

end