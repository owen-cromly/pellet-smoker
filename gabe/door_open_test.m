function door_open_test()
% 2.7.3 — Door-open disturbance at 110°C
% Uses the SAME controller as profile_tracking (pellets + fan, with
% anti-windup + slew limits), and compares:
%   - closed-loop feedback case
%   - simple feed-forward bump (no feedback)
%
% Disturbance: k_ca(t) = 1.5*k_ca for 10 < t < 20 s, else k_ca.

%% Parameters, linearization, operating point
[p,op] = smoker_params();
[A,B,C,~] = smoker_lin(p, op);
eq  = smoker_eq(p, op);
u0  = eq.ue;             % nominal [u_p0; u_f0]
y0  = eq.y0;             % nominal T_c

%% Controller design

poles_ctrl = [-0.2 -0.3 -0.5 -0.6];
poles_obs  = [-0.25 -0.50];

[K,Ki_vec,~] = design_gains(p, op, poles_ctrl, poles_obs);
L           = observer_gain(p, op, poles_obs);
Ki_p        = Ki_vec(1);              % scalar integral gain on pellet channel

% Small-signal steady-state ref→inputs gain (2x1), same R as profile_tracking
Kref = kref_dc(A,B,C,diag([2.5 6.0]));

%% Simulation setup
dt = 0.5;
T  = 300;
TT = (0:dt:T)';
nT = numel(TT);

tau_ref = 100;                         % reference prefilter time constant

% Actuator bounds (match your profile_tracking choices)
up_min  = 0;                        % g/s
up_max  = 2.50;                        % g/s
uf_min  = 0.0;
uf_max  = 5.0;

% Per-step slew limits
du_max_up = 0.05;                   % pellets: smooth
du_max_uf = 0.02;                     % fan: a bit faster
du_max    = [du_max_up; du_max_uf];

% Door disturbance in k_ca
alpha_nom = 1.0;
alpha_hi  = 1.5;                      % 1.5 * k_ca per spec
t_on  = 10;
t_off = 20;

% Target temperature for this test
T_target = 110;                       % we want T_c ≈ 110 °C

%% Initial conditions (start around 110 °C)
% Start both metal and chamber at 110, hopper mass from equilibrium
x0   = [T_target; T_target; eq.xe(3)];

% 1) Closed-loop feedback run
x_fb   = x0;
zhat_fb = zeros(3,1);
xI_fb   = 0;
u_fb    = u0;
r_f_fb  = T_target;

Tc_fb = zeros(nT,1);
UP_fb = zeros(nT,1);
UF_fb = zeros(nT,1);

% 2) Open-loop feed-forward run (for comparison)
x_ff = x0;
u_ff = u0;

Tc_ff = zeros(nT,1);
UP_ff = zeros(nT,1);
UF_ff = zeros(nT,1);

%% Main loop
for k = 1:nT
    t = TT(k);

    % Time-varying k_ca (door open between t_on and t_off)
    if t > t_on && t < t_off
        p_k = p; p_k.kca = alpha_hi * p.kca;
    else
        p_k = p; p_k.kca = alpha_nom * p.kca;
    end

    %% ---------- CLOSED-LOOP FEEDBACK CASE ----------
    % Constant reference r = 110, shaped by first-order prefilter
    r_set   = T_target;
    r_f_fb  = r_f_fb + dt*(r_set - r_f_fb)/tau_ref;

    % Measurements and errors
    y_fb    = x_fb(2);
    ydev_fb = y_fb - y0;
    e_fb    = r_f_fb - y_fb;

    % Feedforward + state feedback + pellet integral
    du_ff_fb  = Kref * (r_f_fb - y0);          % 2x1
    u_int_fb  = -Ki_p * xI_fb;                 % scalar (pellet channel)
    du_fb_vec = -K*zhat_fb + [u_int_fb; 0];    % 2x1

    % Pre-saturation absolute command
    u_unsat_fb = u0 + du_ff_fb + du_fb_vec;    % [u_p; u_f] before limits

    % Static saturation
    up_sat_fb = min(max(u_unsat_fb(1), up_min), up_max);
    uf_sat_fb = min(max(u_unsat_fb(2), uf_min), uf_max);
    u_cmd_fb  = [up_sat_fb; uf_sat_fb];

    % Slew limits → actual commanded inputs
    u_fb = slew_limit(u_fb, u_cmd_fb, du_max); % uses your helper

    % Anti-windup on pellet integrator (8-arg mode)
    xI_fb = anti_windup(xI_fb, e_fb, dt, ...
                        u_unsat_fb(1), ...  % pre-sat pellet command
                        u_fb(1),       ...  % post-sat/post-slew pellet command
                        Ki_p,          ...  % integral gain
                        up_min, up_max);

    % Nonlinear plant step with disturbed k_ca
    x_fb = x_fb + dt * smoker_nl(t, x_fb, @(~)u_fb, p_k);

    % Observer update (same structure as profile_tracking)
    du_dev_fb = [u_fb(1) - u0(1); 0];         % only pellet deviation into B
    zhat_fb   = zhat_fb + dt*(A*zhat_fb + B*du_dev_fb + L*(ydev_fb - C*zhat_fb));

    % Log closed-loop signals
    Tc_fb(k) = y_fb;
    UP_fb(k) = u_fb(1);
    UF_fb(k) = u_fb(2);

    %% ---------- OPEN-LOOP FEED-FORWARD CASE ----------
    % Simple pellet "blip" + modest fan bump; no feedback
    if t > t_on && t < t_off
        % ramp pellets up a bit during door open
        u_ff(1) = min(up_max, u_ff(1) + 0.02/dt);
        % modest fan bump while door open
        u_ff(2) = min(uf_max, u0(2) + 0.02*(t - t_on));
    elseif t >= t_off && t <= t_off + 40
        % ramp pellets and fan back toward nominal
        u_ff(1) = max(u0(1), u_ff(1) - 0.02/dt);
        u_ff(2) = max(u0(2), u_ff(2) - 0.02);
    else
        % back at nominal inputs
        u_ff = u0;
    end

    % Nonlinear plant step with the same disturbed k_ca
    x_ff = x_ff + dt * smoker_nl(t, x_ff, @(~)u_ff, p_k);

    % Log feed-forward signals
    Tc_ff(k) = x_ff(2);
    UP_ff(k) = u_ff(1);
    UF_ff(k) = u_ff(2);
end

%% Plots
fig = figure(3); clf(fig); tiledlayout(fig,2,1);

% Temperature comparison
nexttile;
plot(TT, Tc_fb, 'b-', TT, Tc_ff, 'r--', 'LineWidth', 1.2); hold on
xline(t_on,  '--k', 'Door open');
xline(t_off, '--k', 'Door shut');
grid on
ylabel('T_c (^{\circ}C)');
title('Door-open disturbance rejection (initialized at 110 ^{\circ}C)');
legend('Feedback controller','Feed-forward only','Location','best');

% Pellet input comparison
nexttile;
plot(TT, UP_fb, 'b-', TT, UP_ff, 'r--', 'LineWidth', 1.0); hold on
grid on
ylabel('u_p (g/s)'); xlabel('Time (s)');
legend('u_p (FB)','u_p (FF)','Location','best');

set(fig,'PaperPosition',[0 0 7.2 6.2]);
print(fig,'-dpdf','fig_door_open.pdf');
end