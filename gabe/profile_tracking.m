function profile_tracking(start_at_ambient)
% 2.7.5 — Profile tracking (nonlinear plant; pellet-only control, clean spec version)
%
% If start_at_ambient == true, we start from T_f = T_c = T_amb (cold smoker).
% Otherwise, we start from the equilibrium eq.xe (~110 °C), i.e. a preheated smoker.

if nargin < 1
    start_at_ambient = false;   % default: preheated (matches linearization)
end

[p,op] = smoker_params();                 % helper that returns the §2.6 numbers
[A,B,C,~] = smoker_lin(p, op);            % must match §2.6 linearization in your write-up
eq  = smoker_eq(p, op);
u0  = eq.ue;                               % [up0; uf0]  ~ [0.4; 0.6] per §2.6
y0  = eq.y0;                               % nominal Tc

% Mild state-feedback + observer (designed on §2.6 matrices)
poles_ctrl = [-0.02 -0.03 -0.05 -0.06];   % gentle, well-damped
poles_obs  = [-0.25 -0.50];
[K,Ki_vec,~] = design_gains(p, op, poles_ctrl, poles_obs);
L           = observer_gain(p, op, poles_obs);
Ki_p        = Ki_vec(1);                  % pellet-only integral gain

% Small-signal steady-state feedforward ref→inputs (2x1)
Kref = kref_dc(A,B,C,diag([2.5 6.0]));

% --- SIM / shaping ---
dt      = 0.5; 
T       = 1800;
TT      = (0:dt:T)'; 
nT      = numel(TT);

tau_ref = 50;                % gentle reference prefilter (1st order)
du_max  = 0.0075;            % pellet slew per step (keeps things smooth)
up_min  = 0.12;              % avoid extinguishing fire; not below physical min
up_max  = 1.00;              % reasonable cap
uf_fix  = u0(2);             % keep fan constant for the main spec run

% --- STATE / LOGS ---

if start_at_ambient
    % Cold-start scenario: smoker at ambient, hopper mass as in eq.xe
    % (If your params struct names ambient differently, adjust p.Tamb.)
    Tamb = p.Tamb;           % or 25 if you want to hard-code °C
    xabs = [Tamb; Tamb; eq.xe(3)];
    r_f  = Tc_profile(0);    % start controller ref at first profile segment (90 °C)
else
    % Preheated smoker: start exactly at the nominal equilibrium
    xabs = eq.xe;            % nonlinear plant absolute state
    r_f  = y0;               % filtered reference starts at nominal Tc
end

zhat = zeros(3,1);           % observer state (deviation coords)
xI   = 0;                     % scalar integrator on temperature error
uabs = u0;                    % absolute inputs; we'll move pellets only

Tc_tr = zeros(nT,1);
UP    = zeros(nT,1);
UF    = zeros(nT,1);
Rset  = zeros(nT,1);          % boxy setpoint (90/110/130)
Rshp  = zeros(nT,1);          % shaped ref to controller
MP    = zeros(nT,1);          % hopper mass (to compute pellet usage)
MP(1) = xabs(3);

for k = 1:nT
    t = TT(k);

    % Exact profile the spec asks for (90, 110, 130)
    r_set   = Tc_profile(t);     % 0-600:90, 600-1200:110, 1200-1800:130
    Rset(k) = r_set;

    % Reference shaping (standard prefilter)
    r_f     = r_f + dt*(r_set - r_f)/tau_ref;
    Rshp(k) = r_f;

    % Outputs & errors
    y     = xabs(2);
    ydev  = y - y0;
    e     = r_f - y;

    % Control in deviation coordinates
    du_ff = Kref*(r_f - y0);                    % 2x1 small-signal feedforward
    du_fb = (-K*zhat) + [-Ki_p;0]*e;            % pellet-only integral action

    % Absolute command: pellet moves, fan fixed at nominal per §2.6
    up_des = u0(1) + du_ff(1) + du_fb(1);
    up_des = min(max(up_des, up_min), up_max);

    % Slew-limit pellets for smoothness
    up_cmd = up_slew(uabs(1), up_des, du_max);
    uabs   = [up_cmd; uf_fix];

    % Anti-windup: freeze integrator when hard at bounds and e would push deeper
    at_hi = (abs(up_cmd - up_max) <= 1e-6) && (e > 0);
    at_lo = (abs(up_cmd - up_min) <= 1e-6) && (e < 0);
    if ~(at_hi || at_lo)
        xI = xI + dt*e;
    end
    xI = max(min(xI,200), -200);

    % Plant + observer
    xabs = xabs + dt*smoker_nl(t, xabs, @(~)uabs, p);
    zhat = zhat + dt*(A*zhat + B*[du_fb(1);0] + L*(ydev - C*zhat));

    % Logs
    Tc_tr(k) = y;
    UP(k)    = uabs(1);
    UF(k)    = uabs(2);
    MP(k)    = xabs(3);
end

% Pellet usage (g) and simple step metrics you'll report for 2.7.5
pellets_used = MP(1) - MP(end);

% Plots
fig = figure(5); clf(fig); tiledlayout(fig,2,1)
nexttile;
plot(TT,Rset,'k:',TT,Rshp,'k--',TT,Tc_tr,'b-','LineWidth',1.1); grid on
ylabel('T_c (^{\circ}C)');
if start_at_ambient
    title('Profile tracking (nonlinear plant, cold start)')
else
    title('Profile tracking (nonlinear plant, preheated)')
end
legend('setpoint (boxy)','ref to controller (shaped)','T_c','Location','best')

nexttile;
plot(TT,UP,'-','LineWidth',1.0); hold on
plot(TT,UF,'-','LineWidth',1.0); grid on
ylabel('Inputs'); xlabel('Time (s)');
legend('u_p','u_f','Location','best')

fprintf('Pellets used over profile: %.1f g\n', pellets_used);
end

% --- helpers ---
function up = up_slew(up_prev, up_cmd, du_max)
    du  = up_cmd - up_prev;
    scl = min(1, du_max/max(abs(du),1e-12));
    up  = up_prev + scl*du;
end
