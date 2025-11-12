function u = slew_limit(u_prev, u_cmd, du_max)
% Limit elementwise change per step to +/- du_max.
delta = max(min(u_cmd - u_prev, du_max), -du_max);
u = u_prev + delta;
end
