function [K,Ki,L] = design_gains(p, op, poles_ctrl, poles_obs, opts)
% DESIGN_GAINS  Controller + observer design for smoker case study.
%   [K,Ki,L] = design_gains(p, op, poles_ctrl, poles_obs, opts)
%   - Observer: reduced-order on [Tf,Tc] using "place" (or LQE fallback).
%   - Controller:
%       * Default: continuous-time LQI on [x; xI] (integral of Tc error).
%       * Optional: pellet-only pole placement on augmented [x; xI]
%         when opts.method == 'place'.

[A,B,C,~] = smoker_lin(p, op);
n      = size(A,1);        % 3
p_out  = size(C,1);        % 1  (Tc)

%% ---------- Observer on [Tf,Tc] ----------
A2 = A(1:2,1:2); 
C2 = [0 1];                 % measure Tc

if nargin>=4 && ~isempty(poles_obs) && numel(poles_obs)>=2
    poles2 = poles_obs(1:2);
else
    poles2 = [-0.15 -0.30]; % default observer poles
end

try
    L2 = place(A2', C2', poles2)';    % 2x1
catch
    % fallback Kalman-like design if place fails
    L2 = lqe(A2, eye(2), C2, eye(2), 1, 0);
end
L = [L2; 0];                          % 3x1, mp tracked via up integral

%% ---------- CONTROLLER DESIGN ----------
use_place = (nargin>=5) && isfield(opts,'method') && strcmpi(opts.method,'place');

if use_place
    % =========================================================
    % OPTION 1: pellet-only pole placement on augmented [x; xI]
    % =========================================================
    %
    % Augmented state: xa = [x; xI], where xI = ∫(r - Tc) dt.
    % For design we use r = 0 => xIdot = -C x.
    % Dynamics:
    %   xdot  = A x + B u
    %   xIdot = -C x
    %
    % => Aaug = [A  0;
    %            -C 0],   Bp_aug = [Bp; 0].
    %
    if isempty(poles_ctrl) || numel(poles_ctrl) ~= 4
        error('design_gains(place): poles_ctrl must have 4 entries for [x; xI].');
    end

    Bp     = B(:,1);                     % 3x1 pellet input
    Aaug   = [A, zeros(3,1);             % 4x4
              -C, 0];
    Bp_aug = [Bp; 0];                    % 4x1

    % Check controllability of (Aaug,Bp_aug)
    if rank(ctrb(Aaug,Bp_aug)) < 4
        warning('design_gains(place): (Aaug,Bp_aug) not fully controllable; using LQI fallback instead.');
        use_place = false;               % drop to LQI below
    else
        % 1x4 gain for pellet channel
        Kp_aug = place(Aaug, Bp_aug, poles_ctrl);   % 1x4

        % Map to 2-input [K;Ki]:
        % - First row: pellet gains from place
        % - Second row (fan): zero for simplicity (fan logic handled elsewhere)
        K  = [Kp_aug(1:3); zeros(1,3)];   % 2x3
        Ki = [Kp_aug(4);   0];            % 2x1 (integral acts on pellets only)
        return;
    end
end

% =========================================================
% OPTION 2 (default): LQI on [x; xI]  (your original code)
% =========================================================
if nargin>=5 && isfield(opts,'Q')
    Q = opts.Q;
else
    %           Tf       Tc        mp       xI    (xI = ∫(r - Tc))
    Q = diag([ 2e-3 ,   8e-3 ,    1e-6 ,   1.0 ]);
end

if nargin>=5 && isfield(opts,'R')
    R = opts.R;
else
    %        up   uf   (closer weights ⇒ both inputs share the work)
    R = diag([ 2.5 ,  6.0 ]);
end

epsI = 5e-3;                           % tiny integrator leak
Aaug = [A,           -C';             % 4x4
        zeros(p_out,n), -epsI];
Baug = [B; zeros(p_out,2)];           % 4x2

[Kaug,~,~] = lqr(Aaug, Baug, Q, R);   % 2x4
K  = Kaug(:,1:n);                     % 2x3
Ki = Kaug(:,n+1:end);                 % 2x1  (integral drives both in principle)
end