function yawRates = findYawRates(States)
%FINDYAWRATE Summary of this function goes here
%   Detailed explanation goes here

    S(:,1) = States(:,47) * 1000 / 3600;
    S(:,2) = States(:,48) * 1000 / 3600;
    yawRates = zeros(size(S,1),1);
    
    times = computeDiscretizedTimes(States);


    %t = 17124;%17124;

    
    for t = 1:size(States,1)-1
        LandmarksLeft = zeros(19,2);
        LandmarksRight = zeros(19,2);
        
        counterLeft = 0;
        counterRight = 0;
        
        % Mapping, add landmarks
        left = true;
        for i = 1:19
            sensor = 49 + i;
            angle = ((i-10) / 9) * (0.5 * pi) * -1;

            if States(t,sensor) >= 30
                continue;
            end 

            if i > 1
                previousX = x;
            end
            
            x = States(t, sensor) * cos(angle);
            y = States(t, sensor) * sin(angle);
            
            if left && i > 1 && abs(x - previousX) > 10 
                left = false;
            end
            
            if left
                counterLeft = counterLeft + 1;
                LandmarksLeft(counterLeft,:) = [x y];
            else
                counterRight = counterRight + 1;
                LandmarksRight(counterRight,:) = [x y];
            end
        end

        LandmarksLeft = LandmarksLeft(1:counterLeft,:);
        LandmarksRight = LandmarksRight(1:counterRight,:);
        
        dt = times(t+1) - times(t);
        pos = [0 0];
        move = [S(t,1) S(t,2)] .* dt;

        % Range finder points
        NewLandmarks = zeros(19,2);
        counter = 0;
        for i = 1:19
            sensor = 49 + i;
            angle = ((i-10) / 9) * (0.5 * pi) * -1;

            if States(t+1,sensor) >= 30
                continue;
            end
 
            x = States(t+1, sensor) * cos(angle);% + new_pos(1);
            y = States(t+1, sensor) * sin(angle);% + new_pos(2);
            
            counter = counter + 1;
            NewLandmarks(counter,:) = [x y]; 
        end

        NewLandmarks = NewLandmarks(1:counter,:);
        
        % Localization
        % Search for right yawrate
        yawrate = 0;%-0.007;
        delta = 0.001;

        epsilon = 0.00001;
        [bestError, ResultingPoints] = computeProjectionError(LandmarksLeft, LandmarksRight, move, NewLandmarks, yawrate);

        % Coordinate descent search
        while delta > epsilon
            yawrate = yawrate + delta;

            % Try with greater value
            [error, ResultingPoints] = computeProjectionError(LandmarksLeft, LandmarksRight, move, NewLandmarks, yawrate);

            if error < bestError
                bestError = error;
                delta = delta * 1.1;
            else
                % If no success true smaller value
                yawrate = yawrate - (2 * delta);
                [error, ResultingPoints] = computeProjectionError(LandmarksLeft, LandmarksRight, move, NewLandmarks, yawrate);
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
        
        
        
        yawRates(t) = yawrate / dt;
        
    end
end

