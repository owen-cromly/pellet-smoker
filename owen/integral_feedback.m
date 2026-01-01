% our first feedback design, which takes observer feedback

function u = integral_feedback(x_hat, x_I, lin, setpoint)
%STATIC FEEDBACK Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    x_hat % the observer state estimation value (3x1)
    x_I % % the integral of the measured x_hat v. setpoint error (3x1)
    lin % the linearized system, as a struct
    setpoint % the setpoint for x
end

arguments (Output)
    u % the input
end

% take in the linear system
A2 = lin.A(1:2,1:2);
B2 = lin.B(1:2,1:2);

%k = [place(A2,B2,[-5.8+0.2i; -5.8-0.2i]),[0;0]];
%k = [place(A2,B2,[-5.8; -1.8]),[0;0]];
k = [place(A2,B2,[-0.78+0.002i; -0.78-0.002i]),[0;0]];
%k = [place(A2,B2,[-0.18+0.002i; -0.18-0.002i]),[0;0]];
u = -k*(x_hat-setpoint)    -    [0.1; 0.1]*x_I;
u(2)=1;
%u = [max(u_p_min,min(u_p_max,u_unsat(1))); max(u_f_min,min(u_f_max,u_unsat(2)))];


end