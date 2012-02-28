function theta = findPolicy(model, States, Actions)
%FINDPOLICY Approximate value function
%   THETA = FINDPOLICY(MODEL, STATES) performs fitted value iteration to
%   find an approximation for the value function V as an linear approximation 
%   of the states: V(s) = theta' * phi(s) . 

    discount = 0.99;

    theta = zeros(6, 1);
    
    
    % Number of state samples
    m = size(States,1);
    num_actions = size(Actions,1);
    
    F = phi(States);
    
    for iter = 1:100
        fprintf('iteration %d\n', iter);
        y = zeros(m, 1);
        
        for i = 1:m
            state = States(i,:)';
            q = zeros(num_actions,1);
            
            for a = 1:num_actions
                action = Actions(a,:)';
                
                s_prime = state;
                for k = 1:5
                    s_prime = model.A * mapStates(s_prime')' + model.B * action;
                end
                
                q(a) = reward(state) + discount * (theta' * phi(s_prime')');
            end
            
            y(i) = max(q);
        end
        
        theta = linearRegression(F, y);
        J = computeCost(F, y, theta);
    end
end

