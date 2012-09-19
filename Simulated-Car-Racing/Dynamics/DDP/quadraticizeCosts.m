function [Q, R] = quadraticizeCosts(g, h, x_ref, u_ref, dt, my_eps)
%APPROXIMATECOST Summary of this function goes here
%   Detailed explanation goes here

    x_eps = my_eps;
    u_eps = my_eps;

    % First we approximate the Q matrix
    % Compute g_x with finite difference method
    for i = 1:length(x_ref)
        x_plus = x_ref;
        x_plus(i) = x_plus(i) + x_eps;
        x1_plus = g(x_plus);
        x_minus = x_ref;
        x_minus(i) = x_minus(i) - x_eps;
        x1_minus = g(x_minus);
        Q(:,i) = (x1_plus - x1_minus) / (2*x_eps);
    end

    % Now partial derivative g_{xx}
    
    % Constant term
    q_0 = g(x_ref);
    
    % Now the same for the R matrix
    for i = 1:length(u_ref)
        u_plus = u_ref;
        u_plus(i) = u_plus(i) + u_eps;
        x1_plus = g(u_plus);
        u_minus = u_ref;
        u_minus(i) = u_minus(i) - u_eps;
        x1_minus = g(u_minus);
        R(:,i) = (x1_plus - x1_minus) / (2*u_eps);
    end
end

