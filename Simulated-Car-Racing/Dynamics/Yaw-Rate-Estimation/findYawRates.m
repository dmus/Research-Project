function [yawRates, LeftEdge, RightEdge, Positions] = findYawRates(States)
%FINDYAWRATE Summary of this function goes here
%   Detailed explanation goes here
    
    % Compensation parameter 
    alpha = 0.805;

    S(:,1) = States(:,47) * 1000 / 3600;
    S(:,2) = States(:,48) * 1000 / 3600;
    yawRates = zeros(size(S,1),1);
    
    LeftEdge = zeros(size(S,1)*19,2);
    RightEdge = zeros(size(S,1)*19,2);
    Positions = zeros(0,2);
    
    numLeftLandmarks = 0;
    numRightLandmarks= 0;
    
    times = computeDiscretizedTimes(States);

    t = 18139;%40;%536;
    yaw = 0;
    translation = [0 0];
    %for t = 536:566
    indices = 1:19;
    angles = transp(((indices - 10) / 9) * (0.5*pi) * -1);
    ranges = States(1, indices + 49)';
    Points = [ranges .* cos(angles) ranges .* sin(angles)];
    [PointsLeft, PointsRight] = groupRangeFinders(Points, 10);
    yawrate = 0;
    for t = 1:size(States,1)-1
        fprintf('t = %0.3f\n', times(t));
        
        % Mapping, add landmarks
        LandmarksLeft = PointsLeft;
        LandmarksRight = PointsRight;
        %[LandmarksLeft, LandmarksRight] = groupRangeFinders(Landmarks, 10);

        % Compute move
        dt = times(t+1) - times(t);
        move = [.5*(S(t,1)+S(t+1,1)) .5*(S(t,2)+S(t+1,2))] .* dt;

        % Range finder points
        newRanges = States(t+1, indices + 49)';

        Points = [newRanges .* cos(angles) newRanges .* sin(angles)];
        [PointsLeft, PointsRight] = groupRangeFinders(Points, 10);
        
        % Check if last step before finish
        if States(t+1,4) < States(t,4)-100
            yawrate = 0; % No better estimation available
        else
            % Localization
            % Search for right yawrate
            yawrate = 0;%0.0145
            delta = 0.001;

            epsilon = 0.00001;
            
            %yawrate = fminunc(@(x) computeProjectionError(LandmarksLeft, LandmarksRight, move, PointsLeft, PointsRight, x), yawrate, optimset('LargeScale', 'off', 'TolX', epsilon, 'Display', 'off'));
            yawrate = fminsearch(@(x) computeProjectionError(LandmarksLeft, LandmarksRight, move, PointsLeft, PointsRight, x), yawrate);


            %yawrate = yawrate * alpha;
        end

        
%             R = [cos(yawrate) -sin(yawrate); sin(yawrate) cos(yawrate)];
%             %ResultingPoints = (R * bsxfun(@plus, Points, move)')';
%             ResultingPoints = bsxfun(@plus, R *[PointsLeft;PointsRight]', move')';
%             figure(1);
%             plotScans([LandmarksLeft;LandmarksRight], ResultingPoints, pos, move);
%         
        
        
        yawRates(t) = (yawrate * alpha) / dt;
        
%         R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
%         
%         offset = numLeftLandmarks+1;
%         LeftEdge(offset:offset+size(LandmarksLeft,1)-1,:) = bsxfun(@plus, R * LandmarksLeft', translation')';
%         numLeftLandmarks = numLeftLandmarks + size(LandmarksLeft,1);
%         
%         offset = numRightLandmarks+1;
%         RightEdge(offset:offset+size(LandmarksRight,1)-1,:) = bsxfun(@plus, R * LandmarksRight', translation')';
%         numRightLandmarks = numRightLandmarks + size(LandmarksRight,1);
%         
%         Positions = [Positions; translation];
%         
% %          figure(2);
% %          plotMap(Positions, LeftEdge(1:numLeftLandmarks,:), RightEdge(1:numRightLandmarks,:));
% %         
%         translation = translation + (R * move')';
%         yaw = yaw + yawrate;
    end
end

