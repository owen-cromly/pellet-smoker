function [u, xI, parts] = ctrl_step_dual(r, y, zhat, xI, u_eq, K, Ki, dt, umin, umax)
%CTRL_STEP_DUAL One control step for 2-input plant [u_p; u_f].
% Integral action on pellet valve only (ch.1). Fan has no integral term.
%
% r      : reference (scalar, Tc target)
% y      : measured Tc (scalar)
% zhat   : observer state in centered coordinates (nx-by-1)
% xI     : scalar integrator state (pellet channel)
% u_eq   : 2x1 nominal input [u_p0; u_f0]
% K      : 2xN state-feedback gain (rows map to channels)
% Ki     : scalar integral gain for pellet channel
% dt     : step size (s)
% umin, umax : 2x1 actuator bounds

if nargin < 10, umax = [1;1]; end
if nargin < 9,  umin = [0;0]; end

% 1) error
e = r - y;

% 2) pre-saturation command
u_nom   = u_eq(:);         % 2x1
u_fb    = -K * zhat(:);    % 2x1
u_int   = [-Ki * xI; 0];   % integral only on pellet (ch.1)
u_unsat = u_nom + u_fb + u_int;

% 3) saturation
u = min(max(u_unsat, umin), umax);

% 4) anti-windup for pellet channel
xI = anti_windup(xI, e, dt, u_unsat(1), u(1), Ki, umin(1), umax(1));

if nargout > 2
    parts = struct('e',e,'u_nom',u_nom,'u_fb',u_fb,'u_int',u_int, ...
                   'u_unsat',u_unsat,'u_sat',u);
end
end
