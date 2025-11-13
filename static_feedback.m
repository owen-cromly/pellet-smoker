function u = static_feedback(x_hat,setpoint,lin)
%STATIC FEEDBACK Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    x_hat % the observer state estimation value
    setpoint % the setpoint for x
    lin % the linearizes system, as a struct
end

arguments (Output)
    u % the input
end

% take in the linear system
A2 = lin.A(1:2,1:2);
B2 = lin.B(1:2,1:2);

k = [place(A2,B2,[-3.8+0.2i; -3.8-0.2i]),[0;0]];

u_unsat = -k*(x_hat-setpoint);
u = [max(0,min(10,u_unsat(1))); max(0,min(1,u_unsat(2)))];

end