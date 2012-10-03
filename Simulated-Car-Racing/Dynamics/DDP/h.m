function cost = h(u, u_ref, alpha)
%H Summary of this function goes here
%   Detailed explanation goes here
    deviationPenalty = 0;
    if nargin > 1
        deviationPenalty = sum((u - u_ref) .^2);
    end

    cost = sum(0.1 * u);
    cost = (1 - alpha) * cost + alpha * deviationPenalty;
end

