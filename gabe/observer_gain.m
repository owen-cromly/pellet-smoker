function L = observer_gain(p, op, poles2)
[A,~,~,~] = smoker_lin(p, op);
A2 = A(1:2,1:2); C2 = [0 1];
if nargin<3 || numel(poles2)<2, poles2 = [-0.2 -0.4]; end
try
    L2 = place(A2', C2', poles2)';      % 2x1
catch
    Qn = eye(2); Rn = 1; L2 = lqe(A2, eye(2), C2, Qn, Rn, 0);
end
L = [L2; 0];                             % no correction on m_p
end
