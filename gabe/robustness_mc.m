function robustness_mc(start_at_ambient)
% 2.7.6 — Robustness Monte Carlo
%
% If start_at_ambient == true, each run starts at T_amb.
% Otherwise, each run starts from its own equilibrium eq_i.xe.

if nargin < 1
    start_at_ambient = false;
end

rng(1);
[p,op] = smoker_params();
poles_ctrl = [-0.02 -0.03 -0.05 -0.06];
poles_obs2 = [-0.2 -0.4];
[K,Ki,~]   = design_gains(p, op, poles_ctrl, poles_obs2);

N=100; overs=zeros(N,1); settle=NaN(N,1); pellets=zeros(N,1);
tol=1.5; ref=110;

for i=1:N
    % sample parameters
    s=@(v)v*(0.8+0.4*rand);
    p_i=p;
    p_i.Cf  = s(p.Cf);
    p_i.Cc  = s(p.Cc);
    p_i.kf  = s(p.kf);
    p_i.kfa = s(p.kfa);
    p_i.kca = s(p.kca);

    [A_i,B_i,C_i,~] = smoker_lin(p_i, op);
    eq_i = smoker_eq(p_i, op);
    L_i  = observer_gain(p_i, op, 5*[-0.02 -0.03]);

    dt=0.5; T=1200; TT=(0:dt:T)'; 

    % --- initial state / input ---
    if start_at_ambient
        Tamb  = p_i.Tamb;
        xabs  = [Tamb; Tamb; eq_i.xe(3)];
        uabs  = eq_i.ue;       % nominal inputs
    else
        xabs  = eq_i.xe;
        uabs  = eq_i.ue;
    end

    zhat = zeros(3,1);
    xI   = 0;
    ylog = zeros(size(TT));
    uplog= zeros(size(TT));
    du_max = 0.03;
    u_floor = [max(0.10,eq_i.ue(1)); max(0.35,eq_i.ue(2))];

    for k=1:numel(TT)
        y    = xabs(2);
        ydev = y - eq_i.y0;

        du_des = -K*zhat - Ki*xI;
        u_cmd  = max(u_floor, min([1;1], eq_i.ue + du_des));
        uabs   = slew_limit(uabs, u_cmd, du_max);

        xabs = xabs + dt*smoker_nl(TT(k), xabs, @(~)uabs, p_i);
        zhat = zhat + dt*(A_i*zhat + B_i*du_des + L_i*(ydev - C_i*zhat));
        xI   = anti_windup(xI, ref - y, dt, du_des, uabs);

        ylog(k)=y;
        uplog(k)=uabs(1);
    end

    overs(i) = max(0, max(ylog) - ref);
    % first time after which we're inside tol for the last 30s
    win = round(30/dt);
    for k=1:numel(TT)-win
        if all(abs(ylog(k:end) - ref) < tol), settle(i)=TT(k); break; end
    end
    pellets(i) = trapz(TT, uplog);
end

figure(6); clf
subplot(3,1,1); histogram(overs);  title('Overshoot (^{\circ}C)'); grid on
subplot(3,1,2); histogram(settle); title('Settling time (s)');    grid on
subplot(3,1,3); histogram(pellets); title('Pellets used (g)');    grid on
set(gcf,'PaperPosition',[0 0 6.5 7.5]); print('-dpdf','fig_robustness.pdf');
end
