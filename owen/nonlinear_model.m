function xd = nonlinear_model(t, x, u)
    %NONLINEAR_MODEL Wrapper function for the nonlinear model
    %   Detailed explanation goes here
    arguments (Input)
        t % time, used for injecting inputs and such
        x
        u
    end

    arguments (Output)
        xd
    end

    p.C_f = 500;
    p.k_f = 150;
    p.k_ca = 25;
    p.C_c = 2500;
    p.k_fa = 20;
    p.gamma = 1000;
    p.T_amb = 25;

    if t > 140 && t < 150
        p.k_ca = 25*1.5;
    end

    xd = nonlinear_core(p, x, u);
end

function xd = nonlinear_core(p, x, u)
    %NONLINEAR_SYSTEM Core of the nonlinear model
    %   Detailed explanation goes here
    arguments (Input)
        p % parameters, as a struct
        x
        u
    end

    arguments (Output)
        xd
    end

    C_f = p.C_f;
    k_f = p.k_f;
    k_ca = p.k_ca;
    C_c = p.C_c;
    k_fa = p.k_fa;
    gamma = p.gamma;
    T_amb = p.T_amb;

    x_dot_nonlinear = @(x, u) [
        1/C_f * (  -k_f*(x(1)-x(2)) -k_fa*(x(1)-T_amb) +gamma*u(1)*u(2)  );
        1/C_c * (   k_f*(x(1)-x(2)) -k_ca*(x(2)-T_amb)  );
        -u(1);
    ];
    xd = x_dot_nonlinear([x(1) x(2) x(3)], [u(1) u(2)]);
end