%% Define the symbols
syms C_f C_c k_f k_fa k_ca gamma T_amb
syms x_1 x_2 x_3 u_p u_f

param_symbols = [C_f C_c k_f k_fa k_ca gamma T_amb];
param_values = [500 2500 150 20 25 1000 25];
point_symbols = [x_1 x_2 x_3 u_p u_f];

disp("scr_syms: created symbols")
disp("  - syms C_f C_c k_f k_fa k_ca gamma T_amb")
disp("  - syms x_1 x_2 x_3 u_p u_f")