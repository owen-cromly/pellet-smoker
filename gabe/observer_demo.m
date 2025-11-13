function observer_demo(start_at_ambient)
% 2.7.4 Luenberger observer (reduced) on nonlinear plant
%
% If start_at_ambient == true, starts from T_amb; else from equilibrium.

if nargin < 1
    start_at_ambient = false;
end

[p,op] = smoker_params();
[A,B,C,~] = smoker_lin(p, op);
eq = smoker_eq(p, op);
xe = eq.xe; u0 = eq.ue; Tc0 = eq.y0;

poles_ctrl = [-0.02 -0.03 -0.05 -0.06];
poles_obs2 = [-0.2 -0.4];
[K,Ki,~]   = design_gains(p, op, poles_ctrl, poles_obs2);
L          = observer_gain(p, op, poles_obs2);

% --- initial conditions ---
if start_at_ambient
    Tamb = p.Tamb;
    xabs = [Tamb; Tamb; xe(3)];
else
    xabs = xe;
end

r   = 110;          % regulation target
dt  = 0.5;
T   = 600;
TT  = (0:dt:T)';

zhat = zeros(3,1);
xI   = 0;
Tc    = zeros(size(TT));
Tchat = zeros(size(TT));

for k = 1:numel(TT)
    t = TT(k);
    y = xabs(2);
    ydev = y - Tc0;              % measurement deviation

    du_des = -K*zhat - Ki*xI;         % desired deviation
    u_unsat = u0 + du_des;
    uabs = max([0;0], min([1;1], u_unsat));   % saturate to plant limits
    du_eff = uabs - u0;                        % effective deviation sent
    
    xabs = xabs + dt*smoker_nl(t, xabs, @(~)uabs, p);
    zhat = zhat + dt*(A*zhat + B*du_eff + L*(ydev - C*zhat));

    % integral action on true error (as designed)
    xI = xI + dt*(r - y);
    xI = max(min(xI, 2000), -2000);

    Tc(k)    = y;
    Tchat(k) = Tc0 + C*zhat;
end

figure(4); clf
plot(TT,Tc,'-',TT,Tchat,'--','LineWidth',1.2); grid on
xlabel('Time (s)'); ylabel('T_c (^{\circ}C)');
legend('True','Estimated','Location','best');
if start_at_ambient
    title('Observer convergence (nonlinear, cold start)');
else
    title('Observer convergence (nonlinear, preheated)');
end
set(gcf,'PaperPosition',[0 0 6.5 4]); print('-dpdf','fig_observer.pdf');
end
