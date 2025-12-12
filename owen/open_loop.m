function u = open_loop(input_determination_function, setpoint, input)
% OPEN LOOP ('feedback')
%   This function is used, alongside a callback (for modularity) to
%   calculate the input needed to produce a certain output in the nonlinear
%   system.
arguments (Input)
    input_determination_function % a function to be used to calculate input
                                 % based on the nonlinear system (e.g.
                                 % feed-forward, where it is based on
                                 % setpoint. Not that useful, as passing
                                 % input directly will always work)
    setpoint % the setpoint of the system
    input
end

arguments (Output)
    u % the input
end

u = input;%input_determination_function(setpoint);


end