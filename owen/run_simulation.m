function [t, x_aug, u] = run_simulation(p, sim_params, observer_model, feedback_model)
% RUN_SIMULATION Run the differential equation simulation.
%
%   p: struct containing all model parameters
%   sim_params: struct containing initial conditions and simulation parameters:
%       - T_c_start
%       - m_p_start
%       - lin (linearization object - see linearize.m)
%       - setpoint (function handle of time)
%       - door_status (function handle of time)
%       - t_min, t_max
%   observer_model: the model of observer to use (see observer_wrapper.m)
%   feedback_model: the model of feedback to use (see feedback_wrapper.m)

% ACKNOWLEDGEMENT: THIS WAS REFACTORED WITH GENERATIVE AI. I told ChatGPT to
% turn code I had written into a parametric function. This saved a few
% minutes

    arguments
        p
        sim_params
        observer_model
        feedback_model
    end

    % --- Unpack initialization points ---
    T_c_start = sim_params.T_c_start;
    m_p_start = sim_params.m_p_start;

    % --- Unpack linearization ---
    lin = sim_params.lin;

    % --- Unpack parameter timeseries ---
    setpoint = sim_params.setpoint;
    door_status = sim_params.door_status;

    % --- Unpack time limits ---
    t_min = sim_params.t_min;
    t_max = sim_params.t_max;

    % --- Unpack noise model ---
    noise_bandwidth = sim_params.noise_bandwidth; % Hz
    noise_amplitude = sim_params.noise_amplitude; % normalized to y
    noise_times = linspace(t_min, t_max, (t_max-t_min)*noise_bandwidth);
    noise_values = noise_amplitude*randn(size(noise_times));
    noise = @(t) interp1(noise_times, noise_values, t); % 1000 Hz sampling rate of the measuring device, let's say

    % --- Set up initial conditions ---
    x_0 = [Tce2Tfe(p, T_c_start); T_c_start; m_p_start];
    x_hat_0 = [Tce2Tfe(p, T_c_start); T_c_start; m_p_start];
    x_I_0 = 0;
    x_aug_0 = [x_0; x_hat_0; x_I_0];

    % --- Differential equation ---
    x_dot_aug = @(t,x) full_differential_equation( ...
        x, @nonlinear_model, observer_model, feedback_model, ...
        setpoint(t), door_status(t), lin, p, noise(t));

    % --- Solve ODE ---
    [t, x_aug] = ode45(x_dot_aug, [t_min; t_max], x_aug_0);
    x_aug = x_aug'; % get x_augs as column vectors


    % --- Recover control inputs ---
    u = recover_u(feedback_model, linspace(t_min,t_max,size(x_aug,2)), ...
                  x_aug(1:3,:), x_aug(4:6,:), x_aug(7,:), setpoint, p, lin, noise);

end