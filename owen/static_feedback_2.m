% our second feedback design, which takes observer feedback
% x = [
%      T_f          ;T_c          ;m_p    ;   
%      T_f_hat      ;T_c_hat      ;m_p_hat/0    ;
%      T_f_hat_int  ;T_c_hat_int  ;m_p_hat_int    ;
% ]

function u = static_feedback_2(x_hat,x_hat_int, setpoint,lin)
%STATIC FEEDBACK Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    x_hat % the observer state estimation value
    x_hat_int % the integral of x_hat error
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

u_p_min = 0.1;
u_p_max = 10;
u_f_min = 0.1;
u_f_max = 1;

if u(1) < u_p_min
    u(1) = u_p_min;
end
if u(1) > u_p_max
    u(1) = u_p_max;
    % u(2) = u_f_max; % smart compensation
end
if u(2) < u_f_min
    u(2) = u_f_min;
end
if u(2) > u_f_max
    u(2) = u_f_max;
    % smart compensation
end