function [t, x_aug, u] = run_simulation(p, init_points, lin_timeseries, observer_model, feedback_model)
%RUN_SIMULATION Run the differential equation simulation and plot x_augs.
%
%   p: struct containing all model parameters
%   init_points: struct containing initial conditions:
%       - T_c_start
%       - m_p_start
%   lin_timeseries: struct containing:
%       - lin (linearization object)
%       - T_c_set_series (function handle of time)
%       - setpoint (function handle of time)
%       - door_status (function handle of time)
%       - t_min, t_max
%   scenario: string for the plot title

% ACKNOWLEDGEMENT: THIS WAS DRAFTED WITH GENERATIVE AI. I told ChatGPT to
% turn code I had written into a parametric function. This saved a few
% minutes

    arguments
        p struct
        init_points struct
        lin_timeseries struct
        observer_model
        feedback_model
    end

    % --- Unpack initialization points ---
    T_c_start = init_points.T_c_start;
    m_p_start = init_points.m_p_start;

    % --- Unpack linearization/timeseries ---
    lin = lin_timeseries.lin;
    T_c_set_series = lin_timeseries.T_c_set_series;
    setpoint = lin_timeseries.setpoint;
    door_status = lin_timeseries.door_status;
    t_min = lin_timeseries.t_min;
    t_max = lin_timeseries.t_max;

    % --- Set up initial conditions ---
    x_0 = [Tce2Tfe(p, T_c_start); T_c_start; m_p_start];
    x_hat_0 = [Tce2Tfe(p, T_c_start); T_c_start; m_p_start];
    integral_hat_0 = 0;
    % x_aug_0 structure: [x_0; x_hat_0; integral_hat_0]
    x_aug_0 = [x_0; x_hat_0; integral_hat_0];

    % --- Differential equation ---
    x_dot_aug = @(t,x) full_differential_equation( ...
        x, @nonlinear_model, observer_model, feedback_model, ...
        setpoint(t), door_status(t), lin, p);

    % --- Solve ODE ---
    [t, x_aug] = ode45(x_dot_aug, [t_min; t_max], x_aug_0);
    x_aug = x_aug'; % get x_augs as column vectors

    % --- Recover control inputs ---
    u = recover_u(feedback_model, linspace(t_min,t_max,size(x_aug,2)), ...
                  x_aug(4:6,:), x_aug(7,:), setpoint, p, lin);

end