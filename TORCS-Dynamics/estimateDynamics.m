function model = estimateDynamics(Trials)
%ESTIMATEDYNAMICS Estimates system dynamics T using linear regression
%   MODEL = ESTIMATEDYNAMICS(TRIALS) uses all state-action trajectories to
%   estimate system dynamics.

    state_dimension = 5;
    num_state_features = 7;
    num_action_features = 2;
    
    % Create training set
    X = zeros(0,num_state_features + num_action_features);
    y = zeros(0,state_dimension);

    for i = 1:numel(Trials)
        S = Trials{i}.S;
        A = Trials{i}.A;
        
        X = [X; mapStates(S(1:end-1, :)) A(1:end-1, :)];
        y = [y; S(2:end, :)];
    end

    model.A = zeros(state_dimension, num_state_features);
    model.B = zeros(state_dimension, num_action_features);

    % Compute model parameters with linear regression
    for i = 1:size(y,2)
        theta = linearRegression(X,y(:,i));
        model.A(i,:) = theta(1:num_state_features);
        model.B(i,:) = theta(num_state_features + 1:end);
    end
    
    % Fixes
%     model.A(3,[1:2 4:9]) = 0;
%     model.B(1:5,2) = 0;
%     model.B([1:3 5 6],1) = 0;
%     
%     % Fix for angular velocity
    theta = linearRegression(X(:,[7 9]),y(:,5));
    model.A(5,:) = 0;
    model.B(5,:) = 0;
    model.A(5,7) = theta(1);
    model.B(5,2) = theta(2);
end

