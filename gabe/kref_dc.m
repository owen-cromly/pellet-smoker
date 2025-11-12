function Kref = kref_dc(A,B,C,R)
% Weighted steady-state feed-forward gain Kref (2x1) so that
%   du_ff = Kref * (r - y0)
% hits the new steady state for y = Tc.
% Works even when the full A is singular by using the stable thermal block.

if nargin < 4, R = eye(size(B,2)); end
W  = inv(R);

% Stable thermal block (Tf,Tc); pellet-mass state does not enter Tc DC gain
A2 = A(1:2,1:2);          % 2x2, invertible (ad - bc > 0)
B2 = B(1:2,:);            % 2x2 (inputs to Tf/Tc dynamics)
C2 = [0 1];               % y = Tc

G0 = -C2 * (A2 \ B2);     % 1x2 DC gain from du -> y
% Weighted right pseudo-inverse: 2x1
Kref = (W*G0') / (G0*W*G0');
end
