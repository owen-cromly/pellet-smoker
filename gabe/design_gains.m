function [K,Ki,L] = design_gains(p, op, poles_ctrl, poles_obs, opts)
% DESIGN_GAINS  Controller + observer design for smoker case study.
%   [K,Ki,L] = design_gains(p, op, poles_ctrl, poles_obs, opts)
%   - Observer: reduced-order on [Tf,Tc] using "place".
%   - Controller:
%       * Default: continuous-time LQI on [x; xI] (integral of Tc error).
%       * Optional: pellet-only pole placement on augmented [x; xI]
%         when opts.method == 'place'.

[A,B,C,~] = smoker_lin(p, op);
n      = size(A,1);        % 3
p_out  = size(C,1);        % 1  (Tc)

%% Observer on [Tf,Tc]
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
    % Augmented state xa = [x; xI], xIdot = -C x
    Aaug = [A, zeros(3,1);
            -C, 0];
    Baug = [B; zeros(1,2)];           

    if isempty(poles_ctrl) || numel(poles_ctrl) ~= 4
        error('design_gains(place): poles_ctrl must have 4 entries for [x; xI].');
    end

    % Check controllability with BOTH inputs
    if rank(ctrb(Aaug,Baug)) < 4
        warning('design_gains(place): (Aaug,Baug) not fully controllable; using LQI fallback instead.');
    else
        % Multi-input pole placement: Kaug is 2x4
        Kaug = place(Aaug, Baug, poles_ctrl);   % uses both pellets and fan

        % Map back:
        K  = Kaug(:,1:3);   % 2x3 feedback on [Tf Tc mp]
        Ki = Kaug(:,4);     % 2x1 integral gains for up and uf
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