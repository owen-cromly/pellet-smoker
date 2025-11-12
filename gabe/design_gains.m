function [K,Ki,L] = design_gains(p, op, poles_ctrl, poles_obs, opts)
% LQI on [x; xI] (integral of Tc error). Reduced-order observer on [Tf,Tc].
[A,B,C,~] = smoker_lin(p, op);
n = size(A,1); p_out = size(C,1);

% ---------- Observer on [Tf,Tc] ----------
A2 = A(1:2,1:2); C2 = [0 1];
if nargin>=4 && numel(poles_obs)>=2, poles2 = poles_obs(1:2); else, poles2 = [-0.15 -0.30]; end
try
    L2 = place(A2', C2', poles2)';        % 2x1
catch
    L2 = lqe(A2, eye(2), C2, eye(2), 1, 0);
end
L = [L2; 0];

% ---------- LQI (gentler integral, fan participates) ----------
if nargin>=5 && isfield(opts,'Q'), Q = opts.Q; else
    %           Tf      Tc      mp      xI   (xI = ∫(r - Tc))
    Q = diag([ 2e-3 ,  8e-3 ,  1e-6 ,  1.0 ]);
end
if nargin>=5 && isfield(opts,'R'), R = opts.R; else
    %        up   uf   (closer weights ⇒ both inputs share the work)
    R = diag([ 2.5 ,  6.0 ]);
end

epsI = 5e-3;                                % tiny integrator leak
Aaug = [A,           -C';
        zeros(p_out,n), -epsI];
Baug = [B; zeros(p_out,2)];

[Kaug,~,~] = lqr(Aaug, Baug, Q, R);         % 2x4
K  = Kaug(:,1:n);
Ki = Kaug(:,n+1:end);                       % 2x1  (integral drives both)
end
