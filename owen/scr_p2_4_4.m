% Show that the rank of the controllability matrix is 3
A = p2_4_3.A;
B = subs(p2_4_3.B,u_p,0);
p2_4_4.W = [B A*B A*A*B];
p2_4_4.rank = rank(p2_4_4.W);
%disp(p2_4_4.W)
%disp(p2_4_4.rank)
clear A B

% Show determinant of matrix of nonzero columns
Wnew = p2_4_4.W(:,[1,3,5]);
latex(det(Wnew))