function lin = linearize(p, u)
%LINEARIZE Returns lin.{A, B, C, x} where x is the state value associated
%with this input
%   Detailed explanation goes here
arguments (Input)
    p % struct of parameters
    u % a point about which to linearize
end

arguments (Output)
    lin % a struct representing the linearized system
end

% Nonlinear recast to solve for T_f^o and T_c^o
A2x2 = [
    -1/p.C_f*(p.k_fa+p.k_f),            p.k_f/p.C_f;
              p.k_f/p.C_c,  -1/p.C_c*(p.k_ca+p.k_f)
];
b2x2 = [
    p.k_fa*p.T_amb/p.C_f;
    p.k_ca*p.T_amb/p.C_c
];
q2x2 = [
    p.gamma*u(1)*u(2)/p.C_f;
                      0
];

x = A2x2\(-b2x2-q2x2);

syms x_1 x_2 x_3 u_p u_f
nonlinear_model(0, [x(1);x_2;x_3], [u_p; u_f]);

A = jacobian(nonlinear_model(0, [x_1;x_2;x_3], [u_p; u_f]),[x_1;x_2;x_3]);
lin.A = double(subs(A, [x_1 x_2 x_3 u_p u_f], [x(1) x(2) 0 u(1) u(2)]));

B = jacobian(nonlinear_model(0, [x_1;x_2;x_3], [u_p; u_f]),[u_p;u_f]);
lin.B = double(subs(B, [x_1 x_2 x_3 u_p u_f], [x(1) x(2) 0 u(1) u(2)]));
lin.C = double([0 1 0]);
lin.x = [double(x); 0];


A = jacobian(nonlinear_model(0, [x_1;x_2;x_3], [u_p; u_f]),[x_1;x_2;x_3]);
lin.A2 = double(subs(A, [u_p u_f], [u(1) u(2)]));

B = jacobian(nonlinear_model(0, [x_1;x_2;x_3], [u_p; u_f]),[u_p;u_f]);
lin.B2 = double(subs(B, [u_p u_f], [u(1) u(2)]));
lin.C2 = double([0 1 0]);
%lin.x2 = double(x);



end