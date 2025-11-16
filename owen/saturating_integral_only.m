function [x_hat_dot, x_I_dot] = saturating_integral_only(setpoint, y)
% SATURATING INTEGRAL ONLY
%   This function performs integral state management, filling the slot of
%   an observer in the matlab stackup while not being properly an observer
%   itself. At least not in the full/normal sense.

%   This function seeks to improve our integral action by saturating the
%   derivative of the integral (the integral measure cannot increase by
%   more than some maximum rate). The steady state error is on the order of
%   one degree; therefore, any error larger than one degree should be
%   categorized as "not steady-state error" and saturated away
arguments (Input)
    setpoint % the setpoint of the system (used for integrating setpoint error)
    y % T_C
end

arguments (Output)
    x_hat_dot % estimated state
    x_I_dot % the predicted x_hat v. setpoint error, to be integrated
end

x_hat_dot = [0;0;0]; % no observer! x_hat state not of interest if using
x_I_dot = saturate(y - setpoint(2), integral_saturation, -integral_saturation);

% Anti wind-up is NO LONGER NEEDED! That is good, because it was preventing
% us from accumulating integral error over a longer time period.
%x_I_dot = x_I_dot - x_I*0.0001; % anti-wind up 


end