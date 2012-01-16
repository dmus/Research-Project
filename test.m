% Availabe in functions
global n m n_macrocells n_states n_actions;

n = 8; % nxn gridworld
m = 2; % mxm macrocells

success = 0.7;
fail = 1 - success;
discount = 0.90;

n_macrocells = (n / m) ^ 2;
n_states = n ^ 2;
n_actions = 4; % North, East, South, West



num_demonstrations = 10;
num_steps = 100;
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
    Demos(i,:) = trajectory;
    Expectations(i,:) = mu;
end

% Empirical estimate
mu_expert = mean(Expectations);

exp = zeros(0,n_macrocells);

% 1.
Pol{1} = ceil(rand(64,1) * 4);
exp = [exp; feature_expectations(P, discount, D, Pol{1}, num_steps)];
i = 2;

% 2.


fprintf('Done\n');
