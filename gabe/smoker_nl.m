function dx = smoker_nl(t, x, ufun, p)
% x = [Tf; Tc; mp], ufun(t) -> [up; uf]
Tf = x(1); Tc = x(2); % mp = x(3) (unused in dynamics except through up)
u  = ufun(t); up = u(1); uf = u(2);

alpha = 1; % door-open scaling on kca (default none)
if isfield(p,'kca_scale') && ~isempty(p.kca_scale)
    alpha = p.kca_scale(t);
end
kca = alpha * p.kca;

qcomb = p.beta * up * uf;

dTf = ( -p.kf*(Tf - Tc) - p.kfa*(Tf - p.Tamb) + qcomb )/p.Cf;
dTc = (  p.kf*(Tf - Tc) - kca      *(Tc - p.Tamb)        )/p.Cc;
dmp = -up;

dx = [dTf; dTc; dmp];
end
