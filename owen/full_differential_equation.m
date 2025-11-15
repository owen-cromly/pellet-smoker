function x_dot_aug = full_differential_equation(x, system_model, observer_model, feedback_model, setpoint, door_status, lin, p)
% FULL DIFFERENTIAL EQUATION Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    x
    system_model
    observer_model
    feedback_model
    setpoint % function of t
    door_status % function of t
    lin % linearized model
    p
end

arguments (Output)
    x_dot_aug
end

% x_dot_aug form
% x_dot:                1:3
% x_hat_dot:            4:6
% integral_error_dot:   7:9 % actually just 7 (7 = T_c)
% u_hat_dot:           10:11 % unused

% DECOMPOSE augmented state into variables
y = x(2);
x_hat = x(4:6);
integral_hat = x(7);

% Organize feedback parameters
feed_struct.x_hat = x_hat;
feed_struct.integral_hat = integral_hat;
feed_struct.lin = lin;
feed_struct.setpoint = setpoint;
feed_struct.strobe = NaN;
% Call on the feedback model
u = saturate(feedback_wrapper(feedback_model, feed_struct), [p.u_p_max; p.u_f_max], [p.u_p_min; p.u_f_min]);

% Call on the nonlinear system model
x_dot = system_model(door_status, x, u);

% Organize observer parameters
obs_struct.x_hat = x_hat;
obs_struct.integral_hat = integral_hat;
obs_struct.lin = lin;
obs_struct.setpoint = setpoint;
obs_struct.y = y;
obs_struct.u = u;
% Call on the observer model
[x_hat_dot, integral_hat_dot] = observer_wrapper(observer_model, obs_struct); 
% determine u_hat_dot (unused)
% u_hat_dot = [0;0]; % UNUSED   feedback_model(x_hat_dot, integral_hat_dot, lin, setpoint, [1 1 1 0 1 0]).*(u<[p.u_p_max; p.u_f_max]).*(u>[p.u_p_min;p.u_f_min]);
% x_hat, integral_hat, lin, setpoint, strobe

% assignments for x_dot_aug:
x_dot_aug = [
    x_dot;
    x_hat_dot;
    integral_hat_dot;
    % u_hat_dot % unused
];

end