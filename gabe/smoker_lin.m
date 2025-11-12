function [A,B,C,D] = smoker_lin(p, op)
A = [ -(p.kf+p.kfa)/p.Cf,  p.kf/p.Cf,            0;
       p.kf/p.Cc,         -(p.kf+p.kca)/p.Cc,    0;
       0,                  0,                    0 ];
B = [ (p.beta*op.uf)/p.Cf, (p.beta*op.up)/p.Cf;
      0,                   0;
     -1,                   0 ];
C = [0 1 0];
D = zeros(1,2);
end
