function u = open_loop(input_determination_function, setpoint)
% OPEN LOOP ('feedback')
%   This function is used, alongside a callback (for modularity) to
%   calculate the input needed to produce a certain output in the nonlinear
%   system.
arguments (Input)
    input_determination_function % a function to be used to calculate input
                                 % based on the nonlinear system
    setpoint % the setpoint of the system
end

arguments (Output)
    u % the input
end

u = input_determination_function(setpoint);


end