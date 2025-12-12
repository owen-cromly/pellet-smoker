function u = op_point2u(p, setpoint)
% TC2U Gives open-loop input needed to produce a certain ouptut (setpoint).
% This function assumes that u_f = 1 (otherwise it would be ill-defined
% mapping). It also has the nonlinear model hardcoded into it.
arguments (Input)
    p % model parameters as a struct
    setpoint % the desired setpoint ([T_f; T_c; m_p], but m_p is ignored
end

arguments (Output)
    u % the input
end

T_f = setpoint(1);
T_c = setpoint(2);

u_p = ( p.k_f*(T_f-T_c) + p.k_fa*(T_f-p.T_amb) ) / p.gamma;

u = [u_p; 1];

%u = [0;0];

end