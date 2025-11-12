function eq = smoker_eq(p, op)
% Solve steady-state for Tf0, Tc0 given constant up0, uf0.
% Cf,Cc drop out at steady-state (time derivatives = 0).
kf=p.kf; kfa=p.kfa; kca=p.kca; Tamb=p.Tamb; beta=p.beta;
up0=op.up; uf0=op.uf;

Aeq = [-(kf+kfa),  kf;
        kf,       -(kf+kca)];
beq = [-kfa*Tamb - beta*up0*uf0;
       -kca*Tamb];

x = Aeq \ beq;
Tf0 = x(1); Tc0 = x(2);
mp0 = 500;               % choose any starting pellet mass

eq.xe = [Tf0; Tc0; mp0];
eq.ue = [up0; uf0];
eq.y0 = Tc0;
end
