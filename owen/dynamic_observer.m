function x_hat_d = dynamic_observer(y,x_hat_prime,k)
%DYNAMIC OBSERVER 
%   This observer is designed to implement nonlinearity in the state
%   observer through a shifting B matrix that better reflects the actual
%   "shifting B matrix" of the nonlinear system. This is accomplished by
%   tracking inputs u_p and u_f as additional state variables. The purpose
%   of this is to import them into the B matrix (nonlinearity--accuracy)
%   and re-place poles at each control loop (new k values, which depend on
%   B). The result should be lessened overshoot without cost (presuming we
%   are not latency constrained: an assumption I am comfortable making)
arguments (Input)
    y % T_C
    x_hat_prime % the present value of x_hat (solved by ode45), including 
          % the present value of input
    k % the feedback gain matrix
end

arguments (Output)
    x_hat_d % estimated state
end

A = [-0.34 0.3 0;0.06 -0.07 0;0 0 0]; % linearized at an arbitrary point
B = [1 0.2;0 0;-1 0]; % linearized at an arbitrary point
Acut = [-0.34 0.3;0.06 -0.07]; % linearized at an arbitrary point
L = [place(Acut, [0;1], [-3.1 -2.9])'; 0]; % arbitrary poles

%C = [0 1 0];
%[C; C*A; C*A*A]

x_hat_d = [
    A*x_hat_prime+B*u+L*(y-x_hat_prime(2));
    k*(A*x_hat_prime+B*u+L*(y-x_hat_prime(2)))
];

end