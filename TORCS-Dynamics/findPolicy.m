function theta = findPolicy(model, States, Actions)
%FINDPOLICY Approximate value function
%   THETA = FINDPOLICY(MODEL, STATES) performs fitted value iteration to
%   find an approximation for the value function V as an linear approximation 
%   of the states: V(s) = theta' * phi(s) . 

    discount = 0.9;

    theta = zeros(size(States,2), 1);
    
    
    % Number of state samples
    m = size(States,1);
    num_actions = size(Actions,1);
    
    F = phi(States);
    
    for iter = 1:20
        fprintf('iteration %d\n', iter);
        y = zeros(m, 1);
        for i = 1:m
            state = States(i,:)';
            q = zeros(num_actions,1);
            for a = 1:num_actions
                action = Actions(a,:)';
                s_prime = model.A * state + model.B * action;
                q(a) = reward(state) + discount * (theta' * phi(s_prime')');
            end
            
            y(i) = max(q);
        end
        
        theta = linearRegression(F, y)
    end
end

