function model = estimateDynamics(Trials)
%ESTIMATEDYNAMICS Estimates system dynamics T using linear regression
%   MODEL = ESTIMATEDYNAMICS(TRIALS) uses all state-action trajectories to
%   estimate system dynamics.

    % Create training set
    X = zeros(0,7);
    y = zeros(0,5);

    % Indexes for relevant state features:
    % distFromStart, trackPos, angle, speedX, speedY
    feature_ind = [4 69 1 47 48];
    num_state_features = length(feature_ind);

    for i = 1:numel(Trials)
        S = Trials{i}.S;
        A = Trials{i}.A;

        velocity_control = A(1:end-1, 1) - A(1:end-1, 2);

        % Include distFromStart, trackPos, angle, speedX, speedY, velocity
        % control and steering control
        X = [X; S(1:end-1, feature_ind) velocity_control A(1:end-1, 5)];
        y = [y; S(2:end, feature_ind)];
    end

    model.A = zeros(num_state_features);
    model.B = zeros(num_state_features, 2);

    % Compute model parameters with linear regression
    for i = 1:length(feature_ind)
        theta = linearRegression(X,y(:,i));
        model.A(i,:) = theta(1:num_state_features);
        model.B(i,:) = theta(num_state_features + 1:end);
    end
end

