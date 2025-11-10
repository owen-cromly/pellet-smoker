%% Create nonlinear system
Mynonlinear = @(x) 3*x;

y = linspace(1,10,100);
plot(Mynonlinear(y));