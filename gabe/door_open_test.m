function door_open_test()
% 2.7.3 — Nonlinear door-open disturbance (pellet bump avoided; fan steady)
% Here we use a simple 3-state, pellet-only state feedback, so the 3 poles
% you pick actually control the response.

[p,op] = smoker_params();
[A,B,C,~] = smoker_lin(p, op);
eq = smoker_eq(p, op);

x  = eq.xe;                 % absolute state
u  = eq.ue;                 % absolute inputs
y0 = eq.y0;

% timings
dt = 0.5; T = 300; TT = (0:dt:T)'; nT = numel(TT);
t_on  = 10; t_off = 20;

% disturbance: boost chamber leakage during [t_on,t_off]
alpha_nom = 1.0;
alpha_hi  = 1.6;

Tc_hist = zeros(nT,1);
UP = zeros(nT,1); UF = zeros(nT,1);

% -------------------------------------------------------------------------
% CONTROLLER: simple 3-state pellet-only feedback +  integral
% -------------------------------------------------------------------------

% Controller (same as before)
poles_ctrl = [5.1 3 5 6];
poles_obs2 = [-0.2 -0.4];
[K,Ki,~]   = design_gains(p, op, poles_ctrl, poles_obs2);
L          = observer_gain(p, op, poles_obs2);
Ki_p = Ki; if numel(Ki_p)>1, Ki_p = Ki_p(1); end

Bp = B(:,1);                         % pellet input column (3x1)

% *** These 3 poles NOW REALLY MATTER for door-open behavior ***
poles_ctrl = [-0.01 -0.015 -0.02];   % tweak these as you like

% basic check
if rank(ctrb(A,Bp)) < 3
    warning('Thermal subsystem (A,Bp) nearly uncontrollable for door_open_test.');
end

Kp = place(A,Bp,poles_ctrl);        % 1x3 state-feedback on [Tf Tc mp]

Ki_p = 1e-5;                          % set >0 if you want a bit of integral, 0 = no I

% observer: still use your usual gain based on (A,C)
poles_obs2 = [-0.2 -0.4];           % same as before, affects L
L          = observer_gain(p, op, poles_obs2);

zhat = zeros(3,1); 
xI   = 0;                           % scalar integrator (only if Ki_p ~= 0)

du_max  = 0.03;
u_floor = [max(0.10,u(1)); max(0.35,u(2))];  % keep fire/airflow alive

for k=1:nT
    t = TT(k);

    % ---------------------------------------------------------------------
    % Plant disturbance only: door conductance scale
    % ---------------------------------------------------------------------
    if t>=t_on && t<=t_off
        p_k = p; p_k.kca = alpha_hi*p.kca;
    else
        p_k = p; p_k.kca = alpha_nom*p.kca;
    end

    % constant reference (hold nominal temp)
    r = y0;

    % output & centered error
    y    = x(2);
    e    = r - y;
    ydev = y - y0;

    % ---------------------------------------------------------------------
    % 3-state feedback + (optional) scalar integral on Tc error
    % ---------------------------------------------------------------------
    du_fb_p = -Kp*zhat;             % 1x1: pellet command in deviation coords
    du_fb_f = 0;                    % keep fan near nominal for this test

    % if you want integral, let Ki_p > 0 and xI update
    du_fb_p = du_fb_p - Ki_p*xI;    % pellet integral contribution (if nonzero)

    up_unsat = u(1) + du_fb_p;      % absolute pellets
    uf_cmd   = u(2);                % fan: keep near nominal

    % saturate + floor + slew on pellets
    up_cmd = max(u_floor(1), min(1, up_unsat));
    u_cmd  = [up_cmd; max(u_floor(2), min(1, uf_cmd))];
    u      = slew_limit(u, u_cmd, du_max);

    % plant step with disturbed parameters
    x = x + dt*smoker_nl(t, x, @(~)u, p_k);

    % observer on thermal states (centered)
    zhat = zhat + dt*(A*zhat + Bp*du_fb_p + L*(ydev - C*zhat));

    % simple integral update if Ki_p ~= 0
    if Ki_p ~= 0
        % freeze when pellets saturated and error would push deeper
        push_hi = (u(1) >= 1-1e-6)          && (du_fb_p > 0) && (e > 0);
        push_lo = (u(1) <= u_floor(1)+1e-6) && (du_fb_p < 0) && (e < 0);
        if ~(push_hi || push_lo)
            xI = xI + dt*e;
        end
        xI = max(min(xI,200),-200);
    end

    % log
    Tc_hist(k) = y; 
    UP(k)      = u(1); 
    UF(k)      = u(2);
end

% plot
fig = figure(3); clf(fig); tiledlayout(fig,2,1);

nexttile;
plot(TT,Tc_hist,'LineWidth',1.2); grid on; hold on
xline(t_on,'--k','Door open'); xline(t_off,'--k','Door shut');
ylabel('T_c (^{\circ}C)'); title('Door-open disturbance rejection');

nexttile;
plot(TT,UP,'-','LineWidth',1.0); hold on
plot(TT,UF,'-','LineWidth',1.0); grid on
legend('u_p','u_f'); ylabel('Inputs'); xlabel('Time (s)');

set(fig,'PaperPosition',[0 0 7.2 6.2]); 
print(fig,'-dpdf','fig_door_open.pdf');
end