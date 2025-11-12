function r = Tc_profile(t)
% Multi-step cooking profile
if t < 600
    r = 90;
elseif t < 1200
    r = 110;
else
    r = 130;
end
end
