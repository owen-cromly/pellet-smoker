function u = static_feedback(x_hat)
%STATIC FEEDBACK Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    x_hat % the observer state estimation value
end

arguments (Output)
    u % the feedback
end

A = [-0.34 0.3 0;0.06 -0.07 0;0 0 0]; % linearized at an arbitrary point
B = [1 0.2;0 0;-1 0]; % linearized at an arbitrary point
Acut = [-0.34 0.3;0.06 -0.07]; % linearized at an arbitrary point
Bcut = [1 0.2;0 0]; % linearized at an arbitrary point
%disp("fukc")
%place(Acut,Bcut,[-5; -4.2])
k = [place(Acut,Bcut,[-5; -4.2]),[0;0]];
%size(A)
%size(B)
%size(k)
%size(x_hat)
u_unsat = k*(x_hat-[26.0336; 26.0336; 0]);
u = [max(0,u_unsat(1)); max(0,min(1,u_unsat(2)))];

end