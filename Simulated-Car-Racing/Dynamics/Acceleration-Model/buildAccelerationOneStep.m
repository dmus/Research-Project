function [model, S, U, times, Accelerations] = buildAccelerationOneStep(trainrun)
%BUILDACCELERATIONONESTEP Builds a dynamics model.
%   BUILDACCELERATIONONESTEP buils a dynamics model by minimizing the
%   one-step squared prediction error criterion. TRAINRUN is the file
%   containing the state-action trajectory with which the model is build.
%   Additional output are S (States), U (Control inputs), Accelerations and
%   the associated times in TIMES. These additional output can be used for 
%   the Acceleration-Lagged model.

    T = load(trainrun);

    % Remove states and actions before start signal
    States = T.States(T.States(:,2) >= 0,:);
    Actions = T.Actions(T.States(:,2) >= 0,:);

    % Compute time differences
    times = States(:,2) - circshift(States(:,2),1);
    times(1) = 0;

    % Find resets for current lap time
    resets = find(times < 0);
    for i = 1:length(resets)
        ind = resets(i);
        times(ind) = States(ind,8) - States(ind-1,2) + States(ind,2);
    end

    % Absolute times, state 0 starts at 0
    times = cumsum(times);

    % Compute longitudinal and lateral speeds in m/s, angular speed in
    % rad/s
    S(:,1) = States(:,47) * 1000 / 3600;
    S(:,2) = States(:,48) * 1000 / 3600;
    S(:,3) = estimateYawRate(States);

    % Controls, acceleration, braking and an additional 1
    U = [Actions(:, [1 2 5]) ones(size(Actions,1),1)];

    % Compute acclerations and store them in a matrix
    Accelerations = zeros(size(S,1) - 1, size(S,2));

    for t = 1:size(S,1) - 1
        dt = times(t+1) - times(t);

        % Rotate velocity at time t+1 back into the body frame at time t
        yawrate = S(t,3);
        R = [cos(-yawrate) -sin(-yawrate);sin(-yawrate) cos(-yawrate)];
        speedsBackRotated = R * S(t+1,1:2)';
        Accelerations(t,1:2) = (speedsBackRotated' - S(t,1:2)) / dt;

        % Also angular acceleration
        Accelerations(t,3) = (S(t+1,3) - S(t,3)) / dt;
    end

    % We have 4 matrices for our acceleration-model
    Apos = zeros(2,3);
    Bpos = zeros(2,4);
    Arot = zeros(1,3);
    Brot = zeros(1,4);

    % Solve following linear least squares problems:
    % - Acceleration in x-direction, steering control not included
    X = [S(1:end-1,:) U(1:end-1,[1 2 4])];
    y = Accelerations(:,1);
    theta = linearRegression(X, y);
    Apos(1,:) = theta(1:3);
    Bpos(1,[1 2 4]) = theta([4 5 6]);

    % - Acceleration in y-direction
    X = [S(1:end-1,:) U(1:end-1,:)];
    y = Accelerations(:,2);
    theta = linearRegression(X, y);
    Apos(2,:) = theta(1:3);
    Bpos(2,:) = theta(4:7);

    % - Acceleration in angular speed
    X = [S(1:end-1,:) U(1:end-1,:)];
    y = Accelerations(:,3);
    theta = linearRegression(X, y);
    Arot(1,:) = theta(1:3);
    Brot(1,:) = theta(4:7);

    % Ready, put into model
    model.Apos = Apos;
    model.Bpos = Bpos;
    model.Arot = Arot;
    model.Brot = Brot;
end

