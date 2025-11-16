function u = PI_feedback(y, x_I, setpoint)
%STATIC FEEDBACK Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    y % the output
    x_I % the integral of the output error
    setpoint
end

arguments (Output)
    u % the input
end

k_P = [6; 1];
k_I = [0.1; 0];

u = -k_P*(y - setpoint) - k_I*x_I;
%u = [max(u_p_min,min(u_p_max,u_unsat(1))); max(u_f_min,min(u_f_max,u_unsat(2)))];

end