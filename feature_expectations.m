function [mu, trajectory] = feature_expectations(P, discount, D, policy, num_steps)
%FEATURE_EXPECTATIONS Summary of this function goes here
%   Detailed explanation goes here
    trajectory = zeros(num_steps,1);

    cumprob = cumsum(D);
    r = rand();
    s = find(cumprob > r, 1);
    trajectory(1) = s;
    mu = phi(s);
    
    for t = 2:num_steps
        a = policy(s);
        cumprob = cumsum(P(s, :, a));
        r = rand();
        s = find(cumprob > r, 1);
        
        trajectory(t) = s;
        mu = mu + discount ^ (t-1) * phi(s);
    end

end

