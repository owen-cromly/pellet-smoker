function [x_hat_dot, x_I_dot] = integral_only(setpoint, y, x_I)
% INTEGRAL ONLY
%   This function performs integral state management, filling the slot of
%   an observer in the matlab stackup while not being properly an observer
%   itself. At least not in the full/normal sense.
arguments (Input)
    setpoint % the setpoint of the system (used for integrating setpoint error)
    y % T_C
    x_I % for anti-wind up
end

arguments (Output)
    x_hat_dot % estimated state
    x_I_dot % the predicted x_hat v. setpoint error, to be integrated
end

x_hat_dot = [0;0;0]; % no observer! x_hat state not of interest if using
x_I_dot = y - setpoint(2);
x_I_dot = x_I_dot - x_I*0.0001; % anti-wind up 


end