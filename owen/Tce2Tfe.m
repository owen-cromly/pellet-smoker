function T_f_e = Tce2Tfe(p, T_c_e)
%TCE2TFE Gets the proper equilibrium T_f_e by solving from T_c_e
%   Detailed explanation goes here
arguments (Input)
    p % struct of parameters
    T_c_e % equilibrium temperature in the chamber (usually setpoint)
end

arguments (Output)
    T_f_e
end

% solution to equation
T_f_e = p.k_ca*(T_c_e-p.T_amb)/p.k_f+T_c_e;

end