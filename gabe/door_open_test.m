function door_open_test()
% 2.7.3 — Nonlinear door-open disturbance (pellet bump avoided; fan steady)

[p,op] = smoker_params();
[A,~,~,~] = smoker_lin(p, op);
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

% Controller
poles_ctrl = [-0.02 -0.03 -0.05 -0.06];
poles_obs2 = [-0.2 -0.4];
[K,Ki,~]   = design_gains(p, op, poles_ctrl, poles_obs2);
L          = observer_gain(p, op, poles_obs2);
Ki_p = Ki; if numel(Ki_p)>1, Ki_p = Ki_p(1); end

zhat = zeros(3,1); xI = 0;                  % centered observer, scalar I
du_max = 0.03;
u_floor = [max(0.10,u(1)); max(0.35,u(2))]; % keep fire/airflow alive

for k=1:nT
    t = TT(k);

    % door conductance scale (plant disturbance only)
    if t>=t_on && t<=t_off, p_k=p; p_k.kca=alpha_hi*p.kca;
    else,                   p_k=p; p_k.kca=alpha_nom*p.kca;
    end

    % constant reference (hold nominal temp)
    r = y0;

    % output & centered error
    y = x(2); e = r - y; ydev = y - y0;

    % 2×1 deviation command (pellet integral only); keep fan nominal
    du_fb   = -K*zhat + [-Ki_p*xI; 0];
    up_unsat= u(1) + du_fb(1);
    uf_cmd  = u(2);

    % saturate + floor + slew on pellets
    up_cmd = max(u_floor(1), min(1, up_unsat));
    u_cmd  = [up_cmd; max(u_floor(2), min(1, uf_cmd))];
    u      = slew_limit(u, u_cmd, du_max);

    % plant step with disturbed parameters
    x = x + dt*smoker_nl(t, x, @(~)u, p_k);

    % observer (around nominal A)
    zhat = zhat + dt*(A*zhat + [du_fb(1);0;0] + L*(ydev - [0 1 0]*zhat));

    % anti-windup on pellet integrator
    push_hi = (u(1) >= 1-1e-6)          && (up_unsat > u(1)) && (-Ki_p*e > 0);
    push_lo = (u(1) <= u_floor(1)+1e-6) && (up_unsat < u(1)) && (-Ki_p*e < 0);
    if ~(push_hi || push_lo), xI = xI + dt*e; end
    xI = max(min(xI,200),-200);

    % log
    Tc_hist(k) = y; UP(k) = u(1); UF(k) = u(2);
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

set(fig,'PaperPosition',[0 0 7.2 6.2]); print(fig,'-dpdf','fig_door_open.pdf');
end
