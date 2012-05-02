function [model, Accelerations] = buildAccelerationOneStep(S, U, times)
%BUILDACCELERATIONONESTEP Builds a dynamics model.
%   BUILDACCELERATIONONESTEP buils a dynamics model by minimizing the
%   one-step squared prediction error criterion. TRAINRUN is the file
%   containing the state-action trajectory with which the model is build.
%   Additional output are S (States), U (Control inputs), Accelerations and
%   the associated times in TIMES. These additional output can be used for 
%   the Acceleration-Lagged model.

    
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
    
    
    Apos = zeros(2,5);
    Bpos = zeros(2,9);
    Arot = zeros(1,5);
    Brot = zeros(1,9);

    % Solve following linear least squares problems:
    % - Acceleration in x-direction, steering control not included
    
    X = [mapStates(S) mapInputs(U)];
    X = X(1:end-1,:);
    
    y = Accelerations(:,1);
    
    theta = linearRegression(X, y);
    Apos(1,:) = theta(1:5);
    Bpos(1,:) = theta(6:14);

    % - Acceleration in y-direction
    %X = [S(:,2) S(:,2).^2 acc brake sqrt(acc) sqrt(brake) acc.^2 brake.^2
    %acc.^3 brake.^3 S(:,2) S(:,3) (S(:,2).*S(:,3)).^2 U(:,2) U(:,2).^3 U(:,3)];
    y = Accelerations(:,2);
    theta = linearRegression(X, y);
    Apos(2,:) = theta(1:5);
    Bpos(2,:) = theta(6:14);

    % - Acceleration in angular speed
    %X = [S(:,2) S(:,2).^2 acc brake sqrt(acc) sqrt(brake) acc.^2 brake.^2 acc.^3 brake.^3 S(:,2) S(:,3) (S(:,2).*S(:,3)).^2 U(:,2) U(:,2).^3 U(:,3)];
    
    y = Accelerations(:,3);
    theta = linearRegression(X, y);
    Arot(1,:) = theta(1:5);
    Brot(1,:) = theta(6:14);

    % Ready, put into model
    model.Apos = Apos;
    model.Bpos = Bpos;
    model.Arot = Arot;
    model.Brot = Brot;
end

