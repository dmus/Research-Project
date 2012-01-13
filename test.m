n = 128; % nxn gridworld
m = 16;  % mxm macrocells

success = 0.7;
fail = 1 - success;
discount = 0.99;

n_macrocells = (n / m) ^ 2;
n_states = n ^ 2;
n_actions = 4;

% Initial state distribution
D = rand(n_states, 1);
D = D ./ sum(D);

% True reward function
mask = rand(n_macrocells, 1) > 0.9;
R = rand(n_macrocells,1) .* mask;
R = R ./ sum(R);
R = kron(reshape(R,m,m), ones(m,m));
R = repmat(R(:), 1, n_actions); 

% Transition probabilities
P = zeros(n_states,n_states,n_actions);

count = 0;
for col = 1:n
    for row = 1:n
        count = count + 1;
        
        % North
        if row ~= 1
            P(count, count - 1, 1) = success + fail / n_actions;
        else
            P(count, count, 1) = success + fail / n_actions;
        end
        
        if col ~= n
            P(count, count + n, 1) = fail / n_actions;
        else
            P(count, count, 1) = P(count, count, 1) + fail / n_actions;
        end
        
        if row ~= n
            P(count, count + 1, 1) = fail / n_actions;
        else
            P(count, count, 1) = P(count, count, 1) + fail / n_actions;
        end
        
        if col ~= 1
            P(count, count - n, 1) = fail / n_actions;
        else
            P(count, count, 1) = P(count, count, 1) + fail / n_actions;
        end
        
        % East
        if row ~= 1
            P(count, count - 1, 2) = fail / n_actions;
        else
            P(count, count, 2) = fail / n_actions;
        end
        
        if col ~= n
            P(count, count + n, 2) = success + fail / n_actions;
        else
            P(count, count, 2) = P(count, count, 2) + success + fail / n_actions;
        end
        
        if row ~= n
            P(count, count + 1, 2) = fail / n_actions;
        else
            P(count, count, 2) = P(count, count, 2) + fail / n_actions;
        end
        
        if col ~= 1
            P(count, count - n, 2) = fail / n_actions;
        else
            P(count, count, 2) = P(count, count, 2) + fail / n_actions;
        end
        
        % South
        if row ~= 1
            P(count, count - 1, 3) = fail / n_actions;
        else
            P(count, count, 3) = fail / n_actions;
        end
        
        if col ~= n
            P(count, count + n, 3) = fail / n_actions;
        else
            P(count, count, 3) = P(count, count, 3) + fail / n_actions;
        end
        
        if row ~= n
            P(count, count + 1, 3) = success + fail / n_actions;
        else
            P(count, count, 3) = P(count, count, 3) + success + fail / n_actions;
        end
        
        if col ~= 1
            P(count, count - n, 3) = fail / n_actions;
        else
            P(count, count, 3) = P(count, count, 3) + fail / n_actions;
        end
        
        % West
        if row ~= 1
            P(count, count - 1, 4) = fail / n_actions;
        else
            P(count, count, 4) = fail / n_actions;
        end
        
        if col ~= n
            P(count, count + n, 4) = fail / n_actions;
        else
            P(count, count, 4) = P(count, count, 4) + fail / n_actions;
        end
        
        if row ~= n
            P(count, count + 1, 4) = fail / n_actions;
        else
            P(count, count, 4) = P(count, count, 4) + fail / n_actions;
        end
        
        if col ~= 1
            P(count, count - n, 4) = success + fail / n_actions;
        else
            P(count, count, 4) = P(count, count, 4) + success + fail / n_actions;
        end
    end
end

% Solve MDP with value iteration
[V, policy, iter, cpu_time] = mdp_value_iteration (P, R, discount);

% Sample trajectories
cumprob = cumsum(D);
r = rand();
s = find(cumprob > r, 1);

for i = 1:100
    a = policy(s);
    cumprob = cumsum(P(s, :, a));
    r = rand();
    s = find(cumprob > r, 1);
end

%for from = 1:n_states
%    for to = 1:n_states
%        for action = 1:n_actions
%            P(from,to,1) = transitionProb(from,to,action);
%        end
%    end
%end