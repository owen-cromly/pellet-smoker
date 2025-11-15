function sat = saturate(unsat, max, min)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    unsat
    max
    min
end

arguments (Output)
    sat
end

% saturate the output according to the parameters given
max_sat = (unsat>max).*max + (unsat<max).*unsat;
sat = (max_sat<min).*min + (max_sat>min).*max_sat;

end