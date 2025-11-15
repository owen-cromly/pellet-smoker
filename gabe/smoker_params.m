function [p,op] = smoker_params()
% Physical parameters + operating point (nominal)
p = struct( ...
    'Cf',  500, ...   % J/°C
    'Cc',  2500, ...  % J/°C
    'kf',  150, ...   % W/°C
    'kfa', 20,  ...   % W/°C
    'kca', 25,  ...   % W/°C
    'beta',1000, ...   % W/(g/s) -- set below by calibration
    'Tamb',25);       % °C

op = struct('up',0.4, 'uf',0.6);          % nominal pellet & fan

% --- Calibrate beta so Tc(op) hits target ---
Tc_target = 110;  % desired equilibrium chamber temp

%p.beta = calibrate_beta(p, op, Tc_target);
end

% function beta = calibrate_beta(p, op, Tc_target)
% % Solve steady-state: 
% % [-(kf+kfa)  kf ] [Tf] = [-kfa*Tamb - beta*up*uf]
% % [  kf    -(kf+kca)] [Tc]   [-kca*Tamb             ]
% Aeq = [-(p.kf+p.kfa),  p.kf;
%         p.kf,        -(p.kf+p.kca)];
% b0  = [-p.kfa*p.Tamb; -p.kca*p.Tamb];
% b1  = [-op.up*op.uf;  0];                 % multiplies beta
% 
% x0  = Aeq \ b0;                            % solution at beta = 0
% x1  = Aeq \ b1;                            % derivative w.r.t. beta
% Tc0 = x0(2); Tc1 = x1(2);
% beta = (Tc_target - Tc0)/Tc1;              % choose beta so Tc == target
% end
