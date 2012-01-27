% Availabe in functions
global n m num_macrocells num_states num_actions;

addpath('MDPtoolbox');

n = 32; % nxn gridworld
m = 4; % mxm macrocells

discount = 0.99;
epsilon = 0.1;

num_macrocells = (n / m) ^ 2;
num_states = n ^ 2;
num_actions = 4; % North, East, South, West

num_samples = 10;
num_steps = 100;

Expectations = zeros(num_samples, num_macrocells);

% Initial state distribution
D = rand(num_states, 1);
D = D ./ sum(D);

% True reward function
mask = rand(num_macrocells, 1) > 0.9;
r = rand(num_macrocells,1) .* mask;
r = r ./ sum(r);
R = kron(reshape(r,(n/m),(n/m)), ones(m,m));
R = repmat(R(:), 1, num_actions); 

% Transition probabilities
for a = 1:num_actions
    P{a} = sparse([],[],[],num_states,num_states,num_states * (num_actions + 1));
    for from = 1:num_states
        
        P{a}(from, from) = T(from,a,from);
        
        if from > n && from <= num_states - n 
            P{a}(from, from - 1) = T(from,a,from - 1);
            P{a}(from, from + n) = T(from,a,from + n);
            P{a}(from, from + 1) = T(from,a,from + 1);
            P{a}(from, from - n) = T(from,a,from - n);
        elseif from == 1 % Top-left corner
            P{a}(from, from + n) = T(from,a,from + n);
            P{a}(from, from + 1) = T(from,a,from + 1);
        elseif from <= n % Left column
            P{a}(from, from - 1) = T(from,a,from - 1);
            P{a}(from, from + n) = T(from,a,from + n);
            P{a}(from, from + 1) = T(from,a,from + 1);
        elseif from == num_states % Bottom-right corner
            P{a}(from, from - 1) = T(from,a,from - 1);
            P{a}(from, from - n) = T(from,a,from - n);
        elseif from > num_states - n % Right column
            P{a}(from, from - 1) = T(from,a,from - 1);
            P{a}(from, from + 1) = T(from,a,from + 1);
            P{a}(from, from - n) = T(from,a,from - n);
        end
    end
end

% Solve MDP with value iteration
[V, policy, iter, cpu_time] = mdp_value_iteration (P, R, discount);
fprintf('Optimal policy found...\n');

% Sample trajectories from expert policy
fprintf('Sampling demonstration...\n');
mu_expert = feature_expectations(P, discount, D, policy, num_samples, num_steps);

mu = zeros(num_macrocells, 0);
mu_est = zeros(num_macrocells, 0);
w = zeros(num_macrocells, 0);
t = zeros(0,1);

% 1.
Pol{1} = ceil(rand(num_states,1) * 4);
mu(:,1) = feature_expectations(P, discount, D, Pol{1}, num_samples, num_steps);
i = 2;

% 2.
while 1
    if i > 2
        a = (mu(:,i-1) - mu_est(:,i-2))' * (mu_expert - mu_est(:,i-2)); 
        b = (mu(:,i-1) - mu_est(:,i-2))' * (mu(:,i-1) - mu_est(:,i-2));

        mu_est(:,i - 1) = mu_est(:,i - 2) + (a / b) * (mu(:,i - 1) - mu_est(:,i - 2));
    else
        mu_est(:,1) = mu(:,1);
    end
    w(:,i) = mu_expert - mu_est(:,i - 1);
    t(i) = norm(w(:,i), 2);
    w(:,i) = w(:,i) / t(i);
    
    fprintf('t(%d) = %6.4f\n', i, t(i));

    % 3.
    if t(i) <= epsilon
        fprintf('Terminate...');
        break;
    end

    % 4.

    R = kron(reshape(w(:,i),(n/m),(n/m)), ones(m,m));
    R = repmat(R(:), 1, num_actions); 
    [V, Pol{i}, iter, cpu_time] = mdp_value_iteration (P, R, discount);


    % 5.
    mu(:,i) = feature_expectations(P, discount, D, Pol{i}, num_samples, num_steps);

    % 6.
    i = i + 1;
end

fprintf('Done\n');
