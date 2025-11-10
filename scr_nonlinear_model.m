%% Define nonlinear state-space model for the pellet smoker
x_dot_nonlinear = @(x) 1;
disp("created model as x_dot_nonlinear(x_input)")