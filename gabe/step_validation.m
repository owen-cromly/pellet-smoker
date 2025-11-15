% 2.7.1 Linear vs Nonlinear step (deviation coords, small DU)
[p,op] = smoker_params();
[A,B,C,D] = smoker_lin(p, op);
eq = smoker_eq(p, op);  Tc0 = eq.y0; u0 = eq.ue;

t  = (0:0.5:1800)';
DU = 5;                            % 0.10 g/s step (physically small)

% linear (deviation) response
Udev = [DU*ones(numel(t),1), zeros(numel(t),1)];
Ydev = lsim(ss(A,B,C,D), Udev, t, zeros(3,1));

% nonlinear with identical absolute step (and saturation)
u_fun = @(tau) [min(1, max(0, u0(1) + DU*(tau>=0))); u0(2)];
x0 = eq.xe; [tnl, xnl] = ode45(@(tt,xx) smoker_nl(tt,xx,u_fun,p), t, x0);

Tc_lin = Tc0 + Ydev; Tc_nl = xnl(:,2);
figure(1); clf
plot(t,Tc_lin,'-', tnl,Tc_nl,'--','LineWidth',1.2); grid on
xlabel('Time (s)'); ylabel('T_c (^{\circ}C)'); legend('Linear','Nonlinear','Location','best');
title('Model validation: small step in u_p (centered)');
set(gcf,'PaperPosition',[0 0 6.5 4]); print('-dpdf','fig_step_validation.pdf');
