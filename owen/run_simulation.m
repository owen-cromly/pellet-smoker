function [t, x_aug, u] = run_simulation(p, sim_params, observer_model, feedback_model)
% RUN_SIMULATION Run the differential equation simulation.
%
%   p: struct containing all model parameters
%   sim_params: struct containing initial conditions and simulation parameters:
%       - T_c_start
%       - m_p_start
%       - lin (linearization object - see linearize.m)
%       - setpoint (function handle of time)
%       - input ([u_p;u_f] function handle of time --- for open loop)
%       - door_status (function handle of time)
%       - t_min, t_max
%       - time_resolution
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
    if isfield(sim_params, 'setpoint_times') && isfield(sim_params, 'setpoint_values')
        setpoint = @(t) interp1(sim_params.setpoint_times, sim_params.setpoint_values, t);
    else
        setpoint = sim_params.setpoint;
    end
    setpoint = sim_params.setpoint;
    door_status = sim_params.door_status;
    input = sim_params.input;
    
    % custom
    %r_bandwidth = 5; % Hz
    %r_amplitude = 200; % normalized to y
    %r_times = linspace(sim_params.t_min, sim_params.t_max, (sim_params.t_max-sim_params.t_min)*r_bandwidth);
    %r_values = r_amplitude*randn(size(r_times));
    %T_c_set_series = @(t) 80+40*sin(0.01*t);
    %setpoint = @(t) [Tce2Tfe(p,T_c_set_series(t)); T_c_set_series(t); 0];

    % --- Unpack time limits ---
    t_min = sim_params.t_min;
    t_max = sim_params.t_max;

    % --- Unpack noise model ---
    noise_bandwidth = sim_params.noise_bandwidth; % Hz
    noise_amplitude = sim_params.noise_amplitude; % normalized to y
    noise_times = linspace(t_min, t_max, (t_max-t_min)*noise_bandwidth);
    noise_values = noise_amplitude*randn(size(noise_times));
    noise = @(t) interp1(noise_times, noise_values, t); % 1000 Hz sampling rate of the measuring device, let's say

    % --- Unpack precompensator boolean ---
    precomp = sim_params.precomp;

    % --- Set up initial conditions ---
    x_0 = [Tce2Tfe(p, T_c_start); T_c_start; m_p_start];
    x_hat_0 = [Tce2Tfe(p, T_c_start); T_c_start; m_p_start];
    x_I_0 = 0;
    x_aug_0 = [x_0; x_hat_0; x_I_0];

    % --- Differential equation ---
    x_dot_aug = @(t,x) full_differential_equation( ...
        x, @nonlinear_model, observer_model, feedback_model, ...
        setpoint(t), input(t), door_status(t), lin, p, noise(t), precomp);

    % --- Solve ODE ---
    [t, x_aug] = ode45(x_dot_aug, t_min:sim_params.time_resolution:t_max, x_aug_0);
    x_aug = x_aug'; % get x_augs as column vectors


    % --- Recover control inputs ---
    u = recover_u(feedback_model, linspace(t_min,t_max,size(x_aug,2)), ...
                  x_aug(1:3,:), x_aug(4:6,:), x_aug(7,:), setpoint, input, p, lin, noise);

end