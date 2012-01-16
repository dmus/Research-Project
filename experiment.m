% Availabe in functions
global n m n_macrocells n_states n_actions;

n = 16; % nxn gridworld
m = 2; % mxm macrocells

success = 0.7;
fail = 1 - success;
discount = 0.99;

n_macrocells = (n / m) ^ 2;
n_states = n ^ 2;
n_actions = 4; % North, East, South, West



num_demonstrations = 10;
num_steps = 1000;
Demos = zeros(num_demonstrations, num_steps);
Expectations = zeros(num_demonstrations, n_macrocells);

% Initial state distribution
D = rand(n_states, 1);
D = D ./ sum(D);

% True reward function
mask = rand(n_macrocells, 1) > 0.9;
r = rand(n_macrocells,1) .* mask;
r = r ./ sum(r);
R = kron(reshape(r,(n/m),(n/m)), ones(m,m));
R = repmat(R(:), 1, n_actions); 

% Transition probabilities
P = zeros(n_states,n_states,n_actions);

for from = 1:n_states
    for to = 1:n_states
        for a = 1:n_actions
            P(from,to,a) = T(from,a,to);
        end
    end
end

% Solve MDP with value iteration
[V, policy, iter, cpu_time] = mdp_value_iteration (P, R, discount);
fprintf('Optimal policy found...\n');

% Sample trajectories from expert policy
for i = 1:num_demonstrations
    fprintf('Sampling demonstration %d/%d...\n', i, num_demonstrations);
    [mu, trajectory] = feature_expectations(P, discount, D, policy, num_steps);
    Demos(i,:) = trajectory';
    Expectations(i,:) = mu';
end

% Empirical estimate
mu_expert = mean(Expectations)';
epsilon = 0.01;

mu = zeros(n_macrocells, 0);
mu_est = zeros(n_macrocells, 0);
w = zeros(n_macrocells, 0);
t = zeros(0,1);

% 1.
Pol{1} = ceil(rand(n_states,1) * 4);
mu(:,1) = feature_expectations(P, discount, D, Pol{1}, num_steps);
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
    t(i) = norm(mu_expert - mu_est(:,i - 1), 2);

    fprintf('t(i) = %6.2f\n', t(i));

    % 3.
    if t(i) <= epsilon
        fprintf('Terminate...');
        break;
    end

    % 4.

    fprintf('Searching for policy...\n');
    R = kron(reshape(w(:,i),(n/m),(n/m)), ones(m,m));
    R = repmat(R(:), 1, n_actions);
    [V, Pol{i}, iter, cpu_time] = mdp_value_iteration (P, R, discount);


    % 5.
    mu(:,i) = feature_expectations(P, discount, D, Pol{i}, num_steps);

    % 6.
    i = i + 1;
end

fprintf('Done\n');
