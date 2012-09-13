function [Q, R, c] = quadraticizeCost(g, x_ref, u_ref, dt, my_eps, x_ref_tplus1)
%APPROXIMATECOST Summary of this function goes here
%   Detailed explanation goes here

    x_eps = my_eps;
    u_eps = my_eps;

    for i = 1:length(x_ref)
        x_plus = x_ref_next;
        x_plus(i) = x_plus(i) + x_eps;
        x1_plus = f(x_plus, u_ref, dt);
        x_minus = x_ref_next;
        x_minus(i) = x_minus(i) - x_eps;
        x1_minus = f(x_minus, u_ref, dt);
        A(:,i) = (x1_plus - x1_minus) / (2*x_eps);
    end

    for i = 1:length(u_ref)
        u_plus = u_ref;
        u_plus(i) = u_plus(i) + u_eps;
        x1_plus = f(x_ref, u_plus, dt);
        u_minus = u_ref;
        u_minus(i) = u_minus(i) - u_eps;
        x1_minus = f(x_ref, u_minus, dt);
        B(:,i) = (x1_plus - x1_minus) / (2*u_eps);
    end

    c = f(x_ref,u_ref,dt) - x_ref_next;

end

