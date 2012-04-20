function [yawRates, LeftEdge, RightEdge] = findYawRates(States)
%FINDYAWRATE Summary of this function goes here
%   Detailed explanation goes here

    S(:,1) = States(:,47) * 1000 / 3600;
    S(:,2) = States(:,48) * 1000 / 3600;
    yawRates = zeros(size(S,1),1);
    
    leftEdge = zeros(size(S,1)*19,2);
    rightEdge = zeros(size(S,1)*19,2);
    
    numLeftLandmarks = 0;
    numRightLandmarks= 0;
    
    times = computeDiscretizedTimes(States);

    %t = 536;
    yaw = 0;
    translation = [0 0];
    for t = 1:size(States,1)-1
        fprintf('t = %0.3f\n', times(t));
        
        % Mapping, add landmarks
        indices = 1:19;
        angles = transp(((indices - 10) / 9) * (0.5*pi) * -1);
        ranges = States(t, indices + 49)';
        
        Landmarks = [ranges .* cos(angles) ranges .* sin(angles)];
        Landmarks(ranges >= 30,:) = [];
        
        diff_y = abs(Landmarks(1:end-1,2) - Landmarks(2:end,2));
        [maxVal, indMax] = max(diff_y);
        
        LandmarksLeft = Landmarks(1:indMax,:);
        LandmarksRight = flipud(Landmarks(indMax+1:end,:));
        
        % Compute move
        dt = times(t+1) - times(t);
        pos = [0 0];
        move = [S(t,1) S(t,2)] .* dt;

        % Range finder points
        newRanges = States(t+1, indices + 49)';
        
        Points = [newRanges .* cos(angles) newRanges .* sin(angles)];
        Points(ranges >= 30,:) = [];
        
        % Interpolate extra landmarks
        LandmarksLeftX = zeros(size(LandmarksLeft, 1) * 2 - 1, 2);
        LandmarksRightX = zeros(size(LandmarksRight, 1) * 2 - 1, 2);

        % Left
        odd = mod(1:size(LandmarksLeftX,1), 2) == 1;
        even = mod(1:size(LandmarksLeftX,1), 2) == 0;
        
        LandmarksLeftX(odd,:) = LandmarksLeft;
        LandmarksLeftX(even,:) = (LandmarksLeft(1:end-1,:) + LandmarksLeft(2:end,:)) / 2;
        
        % Smoothing, odd indices are fixed
        fixed = mod(1:size(LandmarksLeftX,1), 2) == 1;
        %LandmarksLeftX = smoothCurve(LandmarksLeftX, fixed, 0.01);

        % Same for right
        odd = mod(1:size(LandmarksRightX,1), 2) == 1;
        even = mod(1:size(LandmarksRightX,1), 2) == 0;
        
        LandmarksRightX(odd,:) = LandmarksRight;
        LandmarksRightX(even,:) = (LandmarksRight(1:end-1,:) + LandmarksRight(2:end,:)) / 2;
        
        % Smoothing, odd indices are fixed
        fixed = mod(1:size(LandmarksRightX,1), 2) == 1;
        %LandmarksRightX = smoothCurve(LandmarksRightX, fixed, 0.01);  
        

        
        % Localization
        % Search for right yawrate
        yawrate = 0;%-0.007;
        delta = 0.001;

        epsilon = 0.00001;
        bestError = computeProjectionError(LandmarksLeftX, LandmarksRightX, move, Points, yawrate);

        % Coordinate descent search
        while delta > epsilon
            yawrate = yawrate + delta;

            % Try with greater value
            error = computeProjectionError(LandmarksLeftX, LandmarksRightX, move, Points, yawrate);

            if error < bestError
                bestError = error;
                delta = delta * 1.1;
            else
                % If no success true smaller value
                yawrate = yawrate - (2 * delta);
                error = computeProjectionError(LandmarksLeftX, LandmarksRightX, move, Points, yawrate);
                if error < bestError
                    bestError = error;
                    delta = delta * 1.1;
                else
                    % If no success, reset yawrate and try with smaller steps next
                    % time
                    yawrate = yawrate + delta;
                    delta = delta * 0.9;
                end
            end
        end
        
        
%         R = [cos(yawrate) -sin(yawrate); sin(yawrate) cos(yawrate)];
%         ResultingPoints = (R * bsxfun(@plus, Points, move)')';
%         plotScans(Landmarks, ResultingPoints, pos, move);
        
        yawRates(t) = yawrate / dt;
        
        R = [cos(-yaw) -sin(-yaw); sin(-yaw) cos(-yaw)];
        
        offset = numLeftLandmarks+1;
        LeftEdge(offset:offset+size(LandmarksLeft,1)-1,:) = (R * bsxfun(@plus, LandmarksLeft, translation)')';
        numLeftLandmarks = numLeftLandmarks + size(LandmarksLeft,1);
        
        offset = numRightLandmarks+1;
        RightEdge(offset:offset+size(LandmarksRight,1)-1,:) = (R * bsxfun(@plus, LandmarksRight, translation)')';
        numRightLandmarks = numRightLandmarks + size(LandmarksRight,1);
        
        translation = translation + move;
        yaw = yaw + yawrate;
    end
end

