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
        yawrate = S(t,3) * dt;
        R = [cos(-yawrate) -sin(-yawrate);sin(-yawrate) cos(-yawrate)];
        speedsBackRotated = R * S(t+1,1:2)';
        Accelerations(t,1:2) = (speedsBackRotated' - S(t,1:2)) / dt;

        % Also angular acceleration
        Accelerations(t,3) = (S(t+1,3) - S(t,3)) / dt;
    end

    % We have 4 matrices for our acceleration-model
    numStateFeatures = 6;
    numInputFeatures = 7;
    
    Apos = zeros(2,numStateFeatures);
    Bpos = zeros(2,numInputFeatures);
    Arot = zeros(1,numStateFeatures);
    Brot = zeros(1,numInputFeatures);

    X = [mapStates(S) mapInputs(U,S)];
    X = X(1:end-1,:);
    
    % Solve following linear least squares problems:
    % - Acceleration in x-direction, steering control not included
    
    X_x = X;
    X_x(:,[2 3 4 9:end]) = 0;
    y = Accelerations(:,1);
    
    theta = linearRegression(X_x, y);
    Apos(1,:) = theta(1:numStateFeatures);
    Bpos(1,:) = theta(numStateFeatures+1:end);

    % - Acceleration in y-direction
    %X = [S(:,2) S(:,2).^2 acc brake sqrt(acc) sqrt(brake) acc.^2 brake.^2
    %acc.^3 brake.^3 S(:,2) S(:,3) (S(:,2).*S(:,3)).^2 U(:,2) U(:,2).^3 U(:,3)];
    X_y = X;
    X_y(:,[1 4:6 7 8 12 13]) = 0;
    
    theta = linearRegression(X_y, y);
    Apos(2,:) = theta(1:numStateFeatures);
    Bpos(2,:) = theta(numStateFeatures+1:end);

    % - Acceleration in angular speed
    %X = [S(:,2) S(:,2).^2 acc brake sqrt(acc) sqrt(brake) acc.^2 brake.^2 acc.^3 brake.^3 S(:,2) S(:,3) (S(:,2).*S(:,3)).^2 U(:,2) U(:,2).^3 U(:,3)];
    X_o = X;
    X_o(:,[1 2 5 6 7 8 10 11]) = 0;
    
    y = Accelerations(:,3);
    theta = linearRegression(X_o, y);
    Arot(1,:) = theta(1:numStateFeatures);
    Brot(1,:) = theta(numStateFeatures+1:end);

    % Ready, put into model
    model.Apos = Apos;
    model.Bpos = Bpos;
    model.Arot = Arot;
    model.Brot = Brot;
end

