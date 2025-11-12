function integral_step()
% 2.7.2 — Integral action step tracking (linear model around (x0,u0))

[p,op] = smoker_params();
[A,B,C,~] = smoker_lin(p, op);
eq = smoker_eq(p, op);

% Gains
poles_ctrl = [-0.02 -0.03 -0.05 -0.06];   % mild, real
poles_obs  = [-0.3 -0.5];                  % not used here but consistent
[K,Ki,~] = design_gains(p, op, poles_ctrl, poles_obs);

% DC gain from ref to inputs (use nonsingular 2x2 thermal block)
Kref = kref_dc(A,B,C,diag([2.5 6.0]));     % 2x1

% Simulation
dt = 0.5; T = 1800; TT = (0:dt:T)'; nT = numel(TT);

x  = zeros(3,1);               % deviation state (centered at eq)
xI = 0;                         % scalar integrator
r0 = eq.y0;                     % 110 °C nominal
r1 = r0 + 10;                   % +10 °C step
t_step = 10;

ylog = zeros(nT,1);

% pellet-channel integral gain as scalar
Ki_p = Ki; if numel(Ki_p) > 1, Ki_p = Ki_p(1); end

for k = 1:nT
    t = TT(k);
    r = (t>=t_step)*r1 + (t<t_step)*r0;     % step at 10 s
    e = r - (C*x + eq.y0);                  % scalar error

    % 2×1 input command in deviation coords:
    %   -K*x           -> 2×1
    %   [-Ki_p*xI; 0]  -> 2×1 (integral on pellet channel only)
    %   Kref*(...)     -> 2×1 feed-forward
    du = -K*x + [-Ki_p*xI; 0] + Kref*(r - eq.y0);

    x  = x + dt*(A*x + B*du);
    xI = xI + dt*e;

    ylog(k) = C*x + eq.y0;
end

% Plot
fig = figure(2); clf(fig);
plot(TT, ylog, 'LineWidth',1.2); grid on
ylabel('T_c (^{\circ}C)'); xlabel('Time (s)');
title('Integral action: step tracking (linear, centered)');
set(fig,'PaperPosition',[0 0 7.2 4.2]); print(fig,'-dpdf','fig_integral_step.pdf');
end
