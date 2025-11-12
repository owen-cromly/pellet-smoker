%% Nonlinear recast
try
p2_3.A = [
    -1/C_f*(k_fa+k_f),            k_f/C_f,  0;
              k_f/C_c,  -1/C_c*(k_ca+k_f),  0;
                    0,                  0,  0
];
p2_3.b = [
    k_fa*T_amb/C_f;
    k_ca*T_amb/C_c;
                 0
];
p2_3.q = @(u) [
    gamma*u(1)*u(2)/C_f;
                      0;
                  -u(1)
];
catch
    disp("you probably forgot to define the syms")
end