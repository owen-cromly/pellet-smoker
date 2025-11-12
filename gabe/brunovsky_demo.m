function brunovsky_demo()
% simple linear-model planning demo (see text for derivations)
[p,op]=smoker_params(); [A,B,C,~]=smoker_lin(p,op); eq=smoker_eq(p,op);
a=(p.kf+p.kfa)/p.Cf; b=p.kf/p.Cf; c=p.kf/p.Cc; d=(p.kf+p.kca)/p.Cc;
b1=(p.phi*op.uf0)/p.Cf; b2=(p.phi*op.up0)/p.Cf;
P=[0 0 -1; 0 1 0; c -d 0]; N=[1 0; c*b1 c*b2];
M=[0 0 0; -(a*c+d*c) (c*b+d^2) 0];
dt=0.5; T=1200; TT=(0:dt:T)'; nT=numel(TT);
% plan: +20C cubic over 600 s
t1=600; r=eq.y0 + min(TT/t1,1).^3 * 20;
% pseudo-open-loop pellets via least squares (illustrative)
Phi=zeros(nT,3); Ak=eye(3);
for k=1:nT, Phi(k,:)=C*((eye(3)+dt*A)^k); end
H=zeros(nT,nT); Ak=eye(3);
for i=1:nT, for j=1:i, H(i,j)= C*(Ak*dt*B(:,1)); Ak=(eye(3)+dt*A)*Ak; end, end
up = H \ (r - eq.y0 - Phi*zeros(3,1)); up=max(0,min(1,up));
figure(6); clf; plot(TT,r,'k--','LineWidth',1.1); grid on
title('Brunovsky/motion planning demo (linear model)');
print('-dpdf','fig_brunovsky_demo.pdf');
end