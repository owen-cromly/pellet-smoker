function u = recover_u(feedback_model, time_linspace, x, x_hat, x_I, setpoint, p, lin, noise)
% RECOVER U Gives you the inputs as a timeseries based on your x_hat
% timeseries
arguments (Input)
    feedback_model
    time_linspace % a linspace of time, so that we can recover setpoint(t)
    x
    x_hat
    x_I
    setpoint
    p
    lin
    noise % function of t
end

arguments (Output)
    u % the recovered input values. Timeseries
end

u = zeros(2,1);
setpoint_series = zeros(3,size(time_linspace,2));

for i=1:size(time_linspace,2)
    setpoint_series(:,i) = setpoint(time_linspace(1,i));
    feed_struct.x_hat = x_hat(:,i);
    feed_struct.x_I = x_I(:,i);
    feed_struct.y = x(2,i) * (1+noise(time_linspace(1,i)));
    feed_struct.lin = lin;
    feed_struct.setpoint = setpoint_series(:,i);
    feed_struct.input_determination_function = @(setpoint) op_point2u(p, setpoint_series(:,i));
    u(:,i) = saturate(feedback_wrapper(feedback_model, feed_struct)+op_point2u(p,setpoint_series(:,i)), [p.u_p_max; p.u_f_max], [p.u_p_min; p.u_f_min]);
end






end