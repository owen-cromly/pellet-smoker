function x_dot_aug = full_differential_equation(x, system_model, observer_model, feedback_model, setpoint, input, door_status, lin, p, noise, precomp)
% FULL DIFFERENTIAL EQUATION Used to define the full system differential
% equation of x_aug. This is called usually by run_simulation to define the
% output of an anonymous function that then gets used in ode45 as part of a
% simulation run
arguments (Input)
    x
    system_model
    observer_model
    feedback_model
    setpoint
    input
    door_status
    lin % linearized model
    p
    noise % amount of measurement noise, normalized to y
    precomp % whether or not to include a precompensation
end

arguments (Output)
    x_dot_aug
end

% x_dot_aug FORM
% x_dot:                1:3
% x_hat_dot:            4:6
% integral_error_dot:    7

% DECOMPOSE augmented state into variables
y = x(2)*(1+noise);
x_hat = x(4:6);
x_I = x(7);

% Organize feedback parameters
feed_struct.x_hat = x_hat;
feed_struct.x_I = x_I;
feed_struct.lin = lin;
feed_struct.setpoint = setpoint;
feed_struct.y = y;
feed_struct.input = input;
% including a callback for open loop (never change this unless you want a
% different model for open loop or something)
feed_struct.input_determination_function = @(setpoint) op_point2u(p, setpoint);
%feed_struct.input_determination_function = @(setpoint) [0.6;0.4];

% Call on the feedback model ATTENTION ATTENTION ATTENTION ATTENTION no
% pre-compensator is being used at the moment!!!!!!!!!!!!!!!!!!!!
u = saturate(feedback_wrapper(feedback_model, feed_struct)+op_point2u(p,setpoint), [p.u_p_max; p.u_f_max], [p.u_p_min; p.u_f_min]);
% Call on the nonlinear system model
x_dot = system_model(door_status, x, u);

% Organize observer parameters
obs_struct.x_hat = x_hat;
obs_struct.x_I = x_I;
obs_struct.lin = lin;
obs_struct.setpoint = setpoint;
obs_struct.y = y;
obs_struct.u = u;
% Call on the observer model
[x_hat_dot, x_I_dot] = observer_wrapper(observer_model, obs_struct); 
% determine u_hat_dot (unused)
% u_hat_dot = [0;0]; % UNUSED   feedback_model(x_hat_dot, x_I_dot, lin, setpoint, [1 1 1 0 1 0]).*(u<[p.u_p_max; p.u_f_max]).*(u>[p.u_p_min;p.u_f_min]);
% x_hat, x_I, lin, setpoint, strobe

% assignments for x_dot_aug:
x_dot_aug = [
    x_dot;
    x_hat_dot;
    x_I_dot;
    % u_hat_dot % unused
];

end