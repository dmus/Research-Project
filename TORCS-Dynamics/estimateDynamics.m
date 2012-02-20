function model = estimateDynamics(Trials)
%ESTIMATEDYNAMICS Estimates system dynamics T using linear regression
%   MODEL = ESTIMATEDYNAMICS(TRIALS) uses all state-action trajectories to
%   estimate system dynamics.

    num_state_features = size(Trials{1}.S,2);
    num_action_features = size(Trials{1}.A,2);
    
    % Create training set
    X = zeros(0,num_state_features + num_action_features);
    y = zeros(0,num_state_features);

    for i = 1:numel(Trials)
        S = Trials{i}.S;
        A = Trials{i}.A;
        
        X = [X; S(1:end-1, :) A(1:end-1, :)];
        y = [y; S(2:end, :)];
    end

    model.A = zeros(num_state_features);
    model.B = zeros(num_state_features, 2);

    % Compute model parameters with linear regression
    for i = 1:num_state_features
        theta = linearRegression(X,y(:,i));
        model.A(i,:) = theta(1:num_state_features);
        model.B(i,:) = theta(num_state_features + 1:end);
    end
end

