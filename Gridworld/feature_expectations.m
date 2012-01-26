function mu = feature_expectations(P, discount, D, policy, num_samples, num_steps)
%FEATURE_EXPECTATIONS Summary of this function goes here
%   Detailed explanation goes here
    global num_macrocells;

    Mu = zeros(num_samples, num_macrocells);

    for i = 1:num_samples
        trajectory = zeros(num_steps,1);

        cumprob = cumsum(D);
        r = rand();
        s = find(cumprob > r, 1);
        trajectory(1) = s;
        Mu(i,:) = phi(s)';

        for t = 2:num_steps
            a = policy(s);
            cumprob = cumsum(P(s, :, a));
            r = rand();
            s = find(cumprob > r, 1);

            trajectory(t) = s;
            Mu(i,:) = Mu(i,:) + discount ^ (t-1) * phi(s)';
        end
    end
    
    mu = mean(Mu)';
end

