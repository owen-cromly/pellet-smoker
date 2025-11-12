% 2.7.4 Luenberger observer (reduced) on nonlinear plant
[p,op] = smoker_params();
[A,B,C,~] = smoker_lin(p, op);
eq = smoker_eq(p, op); xe=eq.xe; u0=eq.ue; Tc0=eq.y0;

poles_ctrl = [-0.02 -0.03 -0.05 -0.06];
poles_obs2 = [-0.2 -0.4];
[K,Ki,~]   = design_gains(p, op, poles_ctrl, poles_obs2);
L          = observer_gain(p, op, poles_obs2);

r = 110; dt=0.5; T=600; TT=(0:dt:T)';
xabs = xe; zhat = zeros(3,1); xI = 0; Tc = zeros(size(TT)); Tchat = Tc;

for k=1:numel(TT)
    t = TT(k);
    y = xabs(2); ydev = y - Tc0;    % measurement deviation
    du = -K*zhat - Ki*xI;           % control from estimated deviation
    uabs = u0 + du; uabs = max([0;0], min([1;1], uabs));
    xabs = xabs + dt*smoker_nl(t, xabs, @(~)uabs, p);
    zhat = zhat + dt*(A*zhat + B*du + L*(ydev - C*zhat));  % observer on deltas
    xI   = xI   + dt*(r - y);
    xI = max(min(xI, 2000), -2000);   % simple anti-windup clamp
    Tc(k) = y;  Tchat(k) = Tc0 + C*zhat;
end

figure(4); clf
plot(TT,Tc,'-',TT,Tchat,'--','LineWidth',1.2); grid on
xlabel('Time (s)'); ylabel('T_c (^{\circ}C)'); legend('True','Estimated','Location','best');
title('Observer convergence (nonlinear, centered)');
set(gcf,'PaperPosition',[0 0 6.5 4]); print('-dpdf','fig_observer.pdf');
