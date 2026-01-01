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

k_P = [
    15;  % Kp
    0.2     % Kf
];


k_I = [
    4;   % Ki
    0     % Kif
];




u = k_P.*(ones(2,1)*(setpoint(2)-y)) - k_I*x_I; % subtracting u_op for proper offset
%u = [max(u_p_min,min(u_p_max,u_unsat(1))); max(u_f_min,min(u_f_max,u_unsat(2)))];

end