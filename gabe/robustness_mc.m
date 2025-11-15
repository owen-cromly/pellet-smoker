function robustness_mc(start_at_ambient)
% 2.7.6 — Robustness Monte Carlo for FULL profile tracking
%
% If start_at_ambient == true, each run starts at T_amb.
% Otherwise, each run starts from the nominal equilibrium eq_nom.xe
% (preheated smoker), same as your main profile_tracking().

if nargin < 1
    start_at_ambient = false;
end

rng(1);   % reproducible

% ---------- Nominal design (same as profile_tracking) ----------
[p_nom, op] = smoker_params();
[A_nom, B_nom, C_nom, ~] = smoker_lin(p_nom, op);
eq_nom  = smoker_eq(p_nom, op);
u0      = eq_nom.ue;     % [up0; uf0]
y0      = eq_nom.y0;     % nominal Tc

% State-feedback + observer (nominal)
poles_ctrl = [-0.02 -0.03 -0.05 -0.06];
poles_obs  = [-0.25 -0.50];
[K, Ki_vec, ~] = design_gains(p_nom, op, poles_ctrl, poles_obs);
L_nom        = observer_gain(p_nom, op, poles_obs);
Ki_p         = Ki_vec(1);                 % pellet-channel integral gain

% DC ref→input gain (nominal)
Kref_nom = kref_dc(A_nom, B_nom, C_nom, diag([2.5 6.0]));

% Simulation / shaping
dt      = 0.5;
T       = 1800;
TT      = (0:dt:T)';
nT      = numel(TT);
tau_ref = 50;        % same prefilter as profile_tracking

% Actuator limits (same as profile_tracking)
up_min  = 0.3;
up_max  = 2.5;
uf_min  = 0.0;
uf_max  = 5.0;

du_max_up = 0.0075;
du_max_uf = 0.02;
du_max    = [du_max_up; du_max_uf];

% ---------- Monte Carlo setup ----------
N      = 100;
tol    = 1.5;        % settling band around 130 °C
overs  = zeros(N,1);
settle = NaN(N,1);
pellets= zeros(N,1);

for i = 1:N
    % ---- sample physical parameters (actual plant) ----
    s     = @(v) v*(0.8 + 0.4*rand);   % ±20 %
    p_i   = p_nom;
    p_i.Cf  = s(p_nom.Cf);
    p_i.Cc  = s(p_nom.Cc);
    p_i.kf  = s(p_nom.kf);
    p_i.kfa = s(p_nom.kfa);
    p_i.kca = s(p_nom.kca);
    % NOTE: controller design (K, Ki_p, Kref_nom, L_nom) stays nominal.

    % ---- initial state ----
    if start_at_ambient
        Tamb  = p_i.Tamb;
        xabs  = [Tamb; Tamb; eq_nom.xe(3)];   % cold smoker, same hopper mass
        r_f   = Tc_profile(0);               % first segment (90 °C)
    else
        xabs  = eq_nom.xe;                   % preheated at nominal equilibrium
        r_f   = y0;
    end

    zhat = zeros(3,1);          % observer state (deviation)
    xI   = 0;                   % scalar integrator on Tc error
    uabs = u0;                  % [up; uf]

    Tc_log = zeros(nT,1);
    mp_log = zeros(nT,1);
    mp_log(1) = xabs(3);

    for k = 1:nT
        t = TT(k);

        % --- profile setpoint + shaping ---
        r_set = Tc_profile(t);         % 90 / 110 / 130
        r_f   = r_f + dt*(r_set - r_f)/tau_ref;

        % outputs & errors
        y    = xabs(2);                % actual Tc
        ydev = y - y0;                 % deviation from nominal
        e    = r_f - y;                % tracking error

        % --- control law (same structure as profile_tracking) ---
        du_ff = Kref_nom*(r_f - y0);   % 2x1 feedforward (nominal)
        u_int = -Ki_p * xI;            % pellet integral contribution
        du_fb = -K*zhat + [u_int; 0];  % feedback + integral

        % pre-saturation command
        u_unsat = u0 + du_ff + du_fb;  % [up_unsat; uf_unsat]

        % static saturation
        up_sat = min(max(u_unsat(1), up_min), up_max);
        uf_sat = min(max(u_unsat(2), uf_min), uf_max);
        u_cmd  = [up_sat; uf_sat];

        % slew-limit
        uabs = slew_limit(uabs, u_cmd, du_max);

        % anti-windup for pellet integrator (8-arg mode)
        xI = anti_windup(xI, e, dt, ...
                         u_unsat(1), ...    % pre-sat pellet
                         uabs(1),   ...     % post-sat/slew pellet
                         Ki_p,      ...     % integral gain
                         up_min, up_max);   % pellet limits

        % plant update (nonlinear with perturbed params)
        xabs = xabs + dt*smoker_nl(t, xabs, @(~)uabs, p_i);

        % observer update uses nominal (A_nom,B_nom,C_nom,L_nom)
        du_dev = [uabs(1) - u0(1); 0];    % pellet deviation only
        zhat   = zhat + dt*(A_nom*zhat + B_nom*du_dev + ...
                            L_nom*(ydev - C_nom*zhat));

        % log
        Tc_log(k) = y;
        mp_log(k) = xabs(3);
    end

    % ---------- metrics on the FINAL (130 °C) plateau ----------
    mask_final = TT >= 1200;
    T_final    = TT(mask_final);
    Tc_final   = Tc_log(mask_final);

    % overshoot relative to 130 °C
    overs(i) = max(0, max(Tc_final) - 130);

    % settling: first time after which |Tc-130| < tol for last 30 s
    win = round(30/dt);
    if numel(T_final) > win
        for k = 1:(numel(T_final) - win)
            if all(abs(Tc_final(k:end) - 130) < tol)
                settle(i) = T_final(k);
                break;
            end
        end
    end

    % pellets used (g) from hopper mass state
    pellets(i) = mp_log(1) - mp_log(end);
end

% ---------- plots ----------
figure(6); clf

subplot(3,1,1);
histogram(overs);
title('Overshoot (^{\circ}C)'); grid on
xlabel('max(T_c - 130, 0)');

subplot(3,1,2);
if all(isnan(settle))
    axis off
    text(0.5,0.5,'No runs met settling criterion',...
        'HorizontalAlignment','center','FontSize',10);
    title('Settling time (s)');
else
    good = ~isnan(settle);
    histogram(settle(good));
    title('Settling time (s)'); grid on
    xlabel('Time to enter |T_c - 130| < tol and stay');
end

subplot(3,1,3);
histogram(pellets);
title('Pellets used (g)'); grid on
xlabel('m_p(0) - m_p(T)');

set(gcf,'PaperPosition',[0 0 6.5 7.5]);
print(gcf,'-dpdf','fig_robustness.pdf');
end