function xI = anti_windup(xI, e, dt, varargin)
%ANTI_WINDUP  Conditional anti-windup for a *scalar* integrator xI.
%
% Supports two call styles:
%   Legacy:  xI = anti_windup(xI, e, dt, du_des, uabs)
%     - du_des : desired change in input (vector or scalar)
%     - uabs   : saturated/actual input after limits (vector or scalar)
%     Behavior: freezes the integrator if any channel is saturated and the
%     command is trying to push further into that saturation, *and* the
%     error would reinforce that push.
%
%   New:    xI = anti_windup(xI, e, dt, u_unsat, u_sat, Ki, umin, umax)
%     - u_unsat : pre-saturation command (scalar for the integrator channel)
%     - u_sat   : post-saturation command (same channel)
%     - Ki      : integral gain used in u_int = -Ki*xI  (scalar)
%     - umin, umax : actuator limits (scalars)
%     Behavior: integrates only when it would NOT push further into the
%     active stop.
%
% In both modes, xI is clamp-limited to [-200, 200].

tol = 1e-6;

switch numel(varargin)
    case 2
        % -------- Legacy mode: (du_des, uabs) --------
        du_des = varargin{1};   % vector or scalar
        uabs   = varargin{2};   % vector or scalar

        sat_hi = (uabs >= 1 - tol) & (du_des > 0);
        sat_lo = (uabs <= 0 + tol) & (du_des < 0);

        if ~((any(sat_hi) && e > 0) || (any(sat_lo) && e < 0))
            xI = xI + dt*e;
        end

    case 5
        % -------- New mode: (u_unsat, u_sat, Ki, umin, umax) --------
        u_unsat = varargin{1};  % scalar (channel with integrator)
        u_sat   = varargin{2};  % scalar
        Ki      = varargin{3};  % scalar
        umin    = varargin{4};  % scalar
        umax    = varargin{5};  % scalar

        push_hi = (u_sat >= umax - tol) && (u_unsat > u_sat) && (Ki*e > 0);
        push_lo = (u_sat <= umin + tol) && (u_unsat < u_sat) && (Ki*e < 0);

        if ~(push_hi || push_lo)
            xI = xI + dt*e;
        end

    otherwise
        error('anti_windup: invalid number of arguments.');
end

% modest hard clamp to avoid slow numeric drift
xI = max(min(xI, 200), -200);
end
