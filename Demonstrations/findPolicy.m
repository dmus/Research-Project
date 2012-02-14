function theta = findPolicy(model, States)
%FINDPOLICY Summary of this function goes here
%   Detailed explanation goes here

    discount = 0.99;

    theta = zeros(size(States,2), 1);
    velocities = [0.6 0.3 0 -0.3 -0.6];
    steerings = [1 0 -1];
    
    [x1,x2] = ndgrid(velocities, steerings);
    Actions = [x1(:) x2(:)];
    
    % Number of state samples
    m = size(States,1);
    num_actions = size(Actions,1);
    
    for iter = 1:1000
        y = zeros(m, 1);
        for i = 1:m
            state = States(i,:)';
            q = zeros(num_actions,1);
            for a = 1:num_actions
                action = Actions(a,:)';
                s_prime = model.A * state + model.B * action;
                q(a) = reward(state) + discount * (theta' * s_prime);
            end
            
            y(i) = max(q);
        end
        
        theta = linearRegression(States, y);
    end
end

