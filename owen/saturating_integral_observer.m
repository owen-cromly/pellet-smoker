function [x_hat_dot, integral_hat_dot] = saturating_integral_observer(x_hat, lin, setpoint, y, u)
% SATURATING INTEGRAL OBSERVER 
%   This observer seeks to improve our integral action by saturating the
%   derivative of the integral (the integral measure cannot increase by
%   more than some maximum rate). The steady state error is on the order of
%   one degree; therefore, any error larger than one degree should be
%   categorized as "not steady-state error" and saturated away
arguments (Input)
    x_hat % the present value of x_hat (solved by ode45)
    lin % the linearized system, as a struct
    setpoint % the setpoint of the system (used for integrating setpoint error)
    y % T_C
    u % the present value of input
end

arguments (Output)
    x_hat_dot % estimated state
    integral_hat_dot % the predicted x_hat v. setpoint error, to be integrated
end

A2 = lin.A(1:2,1:2);
L = [place(A2, [0;1], [-3.1 -2.9])'; 0]; % arbitrary poles

% HERE is where the magic happens
integral_saturation = 1;

x_hat_dot = lin.A*x_hat+lin.B*(u)+L*(y-x_hat(2));
integral_hat_dot = saturate(y - setpoint(2), integral_saturation, -integral_saturation);

% Anti wind-up is NO LONGER NEEDED! That is good, because it was preventing
% us from accumulating integral error over a longer time period.
%integral_hat_dot = integral_hat_dot - integral_hat*0.0001; % anti-wind up 


end