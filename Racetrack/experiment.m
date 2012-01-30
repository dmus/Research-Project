%% Racetrack experiment
% Apprenticeship learning experiment in a racetrack gridworld

%%
% Make parameters availabe in functions
global Racetrack num_states num_actions size_statespace num_features;

addpath('MDPtoolbox');

% racetrack (a), 1 means road, 0 = off-road
Racetrack = ones(12,35);
Racetrack(1:5, 1:32) = 0;
Racetrack(10, 1:4) = 0;
Racetrack(11, 1:8) = 0;
Racetrack(12, 1:12) = 0;

discount = 0.99;
epsilon = 0.1;
num_features = 3;

% Number of rows, columns and number of possibilities for horizontal and
% vertical speed
rows = size(Racetrack,1);
columns = size(Racetrack,2);
speeds_ver = 5;
speeds_hor = 5;
size_statespace = [rows columns speeds_ver speeds_hor];
num_states = rows * columns * speeds_ver * speeds_hor;
Actions = [0 0; 1 0; -1 0; 0 1; 0 -1]';

min_state = [1; 1; -2; -2];
max_state = [rows; columns; 2; 2];

num_actions = size(Actions,2);

num_samples = 10; % Number of samples to take to approximate feature expectations
num_steps = 30; % Number of steps for each sample

% Initial state distribution
D = zeros(rows, columns, speeds_ver, speeds_hor);
D(6:9, 1, 3, 3) = 0.25;

% True reward function
R = zeros(12,35,5,5);
for row = 1:rows
    for column = 1:columns
        for speed_ver = 1:speeds_ver
            for speed_hor = 1:speeds_hor
                s = [row;column;speed_ver;speed_hor];
                % Action doesn't matter
                R(row,column,speed_ver,speed_hor) = reward(s, 1);
            end
        end
    end
end

R = repmat(R(:), 1, num_actions);

% Transition probabilities
filter = [0;0;3;3];
for i = 1:num_actions
    a = Actions(:,i);
    P{i} = sparse([],[],[],num_states,num_states,num_states * 6);
    
    for s0_ind = 1:num_states
        [row, column, speed_ver, speed_hor] = ind2sub(size(D), s0_ind);
        s0 = [row;column;speed_ver;speed_hor];
        s0 = s0 - filter;
        
        % Check if valid state
        if ~Racetrack(s0(1), s0(2))
            continue;
        end
        
        % Same state
        s1 = s0;
        temp = s1 + filter;
        P{i}(s0_ind, sub2ind(size(D), temp(1), temp(2), temp(3), temp(4))) = T(s0, a, s1);
                    
        % Same speed
        s1 = s0;
        s1(1:2) = s0(1:2) + s0(3:4);
        if s1 >= min_state & s1 <= max_state
            temp = s1 + filter;
            P{i}(s0_ind, sub2ind(size(D), temp(1), temp(2), temp(3), temp(4))) = T(s0, a, s1);
        end
           
        % Speed reset
        s1 = s0;
        s1(3:4) = 0;
        if s1 >= min_state & s1 <= max_state
            temp = s1 + filter;
            P{i}(s0_ind, sub2ind(size(D), temp(1), temp(2), temp(3), temp(4))) = T(s0, a, s1);
        end
        
        s1 = s0;
        s1(1:2) = s0(1:2) + s0(3:4);
        s1(3:4) = 0;
        if s1 >= min_state & s1 <= max_state
            temp = s1 + filter;
            P{i}(s0_ind, sub2ind(size(D), temp(1), temp(2), temp(3), temp(4))) = T(s0, a, s1);
        end
        
        % Action successful
        s1 = s0;
        s1(3:4) = s0(3:4) + a;
        if s1 >= min_state & s1 <= max_state
            temp = s1 + filter;
            P{i}(s0_ind, sub2ind(size(D), temp(1), temp(2), temp(3), temp(4))) = T(s0, a, s1);
        end
        
        s1 = s0;
        s1(1:2) = s0(1:2) + s0(3:4);
        s1(3:4) = s0(3:4) + a;
        if s1 >= min_state & s1 <= max_state
            temp = s1 + filter;
            P{i}(s0_ind, sub2ind(size(D), temp(1), temp(2), temp(3), temp(4))) = T(s0, a, s1);
        end
    end
end

% Solve MDP with value iteration
[V, policy, iter, cpu_time] = mdp_value_iteration (P, R, discount);
fprintf('Optimal policy found...\n');

% Sample trajectories from expert policy
fprintf('Sampling demonstration...\n');
mu_expert = feature_expectations(P, discount, D, policy, num_samples, num_steps);

mu = zeros(num_features, 0);
mu_est = zeros(num_features, 0);
w = zeros(num_features, 0);
t = zeros(0,1);

% Projection algorithm
% 1.
Pol{1} = ceil(rand(num_states,1) * num_actions);
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

    for j = 1:num_states
        R(j,:) = w(:,i)' * phi(j);
    end
     
    [V, Pol{i}, iter, cpu_time] = mdp_value_iteration (P, R, discount);

    % 5.
    mu(:,i) = feature_expectations(P, discount, D, Pol{i}, num_samples, num_steps);

    % 6.
    i = i + 1;
end

fprintf('Done\n');
