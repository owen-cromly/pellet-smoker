function x_hat_d = static_observer(y,x_hat,u)
%STATIC OBSERVER 
%   Detailed explanation goes here
arguments (Input)
    y % T_C
    x_hat % the present value of x_hat (solved by ode45)
    u % the present value of feedback
end

arguments (Output)
    x_hat_d % estimated state
end

A = [-0.34 0.3 0;0.06 -0.07 0;0 0 0]; % linearized at an arbitrary point
B = [1 0.2;0 0;-1 0]; % linearized at an arbitrary point
Acut = [-0.34 0.3;0.06 -0.07]; % linearized at an arbitrary point
%Bcut = [1 0.2;0 0]; % linearized at an arbitrary point
%disp(size(Acut))
L = [place(Acut',[0 1]',[-3.1; -2.9])';0]; % arbitrary poles

%C = [0 1 0];
%[C; C*A; C*A*A]

x_hat_d = A*x_hat+B*u-L*(y-x_hat(2));

end