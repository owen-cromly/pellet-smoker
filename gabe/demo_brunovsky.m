function demo_brunovsky()
% Brunovský feedback + motion planning (linear, centered, robust)
[p,op] = smoker_params();
[A,B,C,~] = smoker_lin(p, op);
eq  = smoker_eq(p, op); Tc0 = eq.y0;

a=(p.kf+p.kfa)/p.Cf;  b=p.kf/p.Cf;  c=p.kf/p.Cc;  d=(p.kf+p.kca)/p.Cc;

% z = [y1; y2; y2dot] with y1=-mp, y2=Tc
P = [ 0 0 -1; 0 1 0; c -d 0];  E = inv(P);
Az = [0 0 0; 0 0 1; 0 0 0];  Bz = [1 0; 0 0; 0 1];

S = P*B;                 % 3x2
N = [S(1,:); S(3,:)];    % ensures P*B*N^{-1} = Bz
Ninv = inv(N);
R = P*A*E - Az;
ME = [R(1,:); R(3,:)];
M  = ME * P;

% sanity check
Az_est = P*(A - B*(Ninv*M))*E;  Bz_est = P*B*Ninv;
if norm(Az_est-Az,'fro')>1e-8 || norm(Bz_est-Bz,'fro')>1e-8, error('Mapping mismatch'); end

% feedback in z
poles = [-0.02 -0.03 -0.05];
k3 = -(sum(poles)); k2 = poles(1)*poles(2)+poles(1)*poles(3)+poles(2)*poles(3); k1 = -prod(poles);
Kz = [0 1 0; k1 k2 k3];
Kx = -(Ninv*(Kz*P + M));   % u = Kx * x_dev

% (i) feedback-only (should stay near Tc0)
xdev=zeros(3,1); dt=1; T=1200; TT=(0:dt:T)'; Tc_fb=zeros(size(TT));
for k=1:numel(TT)
    du = Kx*xdev;
    xdev = xdev + dt*(A*xdev + B*du);
    Tc_fb(k) = Tc0 + C*xdev;
end

% (ii) open-loop plan: **quintic** with y2, y2dot, y2ddot = 0 at both ends
tau=300; y10=0; y11=-30;           % small pellet deviation
y20=0;  y21=+20; dy20=0; dy21=0; ddy20=0; ddy21=0;
coef = quintic_through(y20,dy20,ddy20,y21,dy21,ddy21,tau);

xdev=zeros(3,1); Tc_plan=zeros(size(TT));
for k=1:numel(TT)
    t = min(TT(k),tau); s=t;
    v1 = (y11-y10)/tau;                 % dot y1 (constant during plan)
    [~,~,y2dd] = eval_quintic(coef,s);  % ddot y2
    if TT(k)>tau, v1=0; y2dd=0; end     % rest after plan
    v  = [v1; y2dd];
    du = Ninv*(v - M*xdev);
    xdev = xdev + dt*(A*xdev + B*du);
    Tc_plan(k) = Tc0 + C*xdev;
end

figure(7); clf
plot(TT,Tc_fb,'-',TT,Tc_plan,'--','LineWidth',1.1); grid on
xlabel('Time (s)'); ylabel('T_c (^{\circ}C)'); legend('Feedback-only','Open-loop planned','Location','best');
title('Brunovsky feedback and motion planning (linear, centered)');
set(gcf,'PaperPosition',[0 0 6.5 4]); print('-dpdf','fig_brunovsky_demo.pdf');
end

function c = quintic_through(y0,dy0,ddy0,y1,dy1,ddy1,T)
% y = a0+a1 t+a2 t^2+a3 t^3+a4 t^4+a5 t^5 with pos/vel/acc constraints at 0 and T
A = [ 1  0   0     0      0       0;
      0  1   0     0      0       0;
      0  0   2     0      0       0;
      1  T  T^2   T^3    T^4     T^5;
      0  1  2*T  3*T^2  4*T^3   5*T^4;
      0  0   2   6*T   12*T^2  20*T^3];
b = [y0; dy0; ddy0; y1; dy1; ddy1];
c = A\b;
end

function [y,yd,ydd] = eval_quintic(c,t)
y   = c(1)+c(2)*t+c(3)*t^2+c(4)*t^3+c(5)*t^4+c(6)*t^5;
yd  = c(2)+2*c(3)*t+3*c(4)*t^2+4*c(5)*t^3+5*c(6)*t^4;
ydd = 2*c(3)+6*c(4)*t+12*c(5)*t^2+20*c(6)*t^3;
end
