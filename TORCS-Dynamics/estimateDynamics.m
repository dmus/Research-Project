function model = estimateDynamics(Trials)
%ESTIMATEDYNAMICS Estimates system dynamics T using linear regression
%   MODEL = ESTIMATEDYNAMICS(TRIALS) uses all state-action trajectories to
%   estimate system dynamics.

    num_state_features = 8;
    num_action_features = 2;
    
    % Create training set
    X = zeros(0,num_state_features + num_action_features);
    y = zeros(0,6);

    for i = 1:numel(Trials)
        S = Trials{i}.S;
        A = Trials{i}.A;
        
        X = [X; mapStates(S(1:end-1, :)) A(1:end-1, :)];
        y = [y; S(2:end, :)];
    end

    model.A = zeros(6, num_state_features);
    model.B = zeros(6, num_action_features);

    % Compute model parameters with linear regression
    for i = 1:size(y,2)
        theta = linearRegression(X,y(:,i));
        model.A(i,:) = theta(1:num_state_features);
        model.B(i,:) = theta(num_state_features + 1:end);
    end
end

