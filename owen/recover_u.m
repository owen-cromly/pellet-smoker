function u = recover_u(feedback_model, time_linspace, x_hat, integral_hat, setpoint, p, lin)
% RECOVER U Gives you the inputs as a timeseries based on your x_hat
% timeseries
arguments (Input)
    feedback_model
    time_linspace % a linspace of time, so that we can recover setpoint(t)
    x_hat
    integral_hat
    setpoint
    p
    lin
end

arguments (Output)
    u % the recovered input values. Timeseries
end

u = zeros(2,1);
setpoint_series = zeros(3,size(time_linspace,2));

for i=1:size(time_linspace,2)
    setpoint_series(:,i) = setpoint(time_linspace(1,i));
    feed_struct.x_hat = x_hat(:,i);
    feed_struct.integral_hat = integral_hat(:,i);
    feed_struct.lin = lin;
    feed_struct.setpoint = setpoint_series(:,i);
    u(:,i) = saturate(feedback_wrapper(feedback_model, feed_struct), [p.u_p_max; p.u_f_max], [p.u_p_min; p.u_f_min]);
end






end