function yawRates = findYawRates(States)
%FINDYAWRATE Summary of this function goes here
%   Detailed explanation goes here

    S(:,1) = States(:,47) * 1000 / 3600;
    S(:,2) = States(:,48) * 1000 / 3600;
    yawRates = zeros(size(S,1),1);
    
    times = States(:,2) - circshift(States(:,2),1);
    times(1) = 0;

    % Find resets for current lap time
    resets = find(times < 0);
    for i = 1:length(resets)
        ind = resets(i);
        times(ind) = States(ind,8) - States(ind-1,2) + States(ind,2);
    end

    times = cumsum(times);


    %t = 17124;%17124;

    
    for t = 1:size(States,1)-1
        Landmarks = zeros(0,2);
        
        % Mapping
        for i = 1:19
            sensor = 49 + i;
            angle = ((i-10) / 9) * (0.5 * pi) * -1;

            if States(t,sensor) >= 100
                continue;
            end

            x = States(t, sensor) * cos(angle);
            y = States(t, sensor) * sin(angle);
            Landmarks = [Landmarks; [x y]];
        end

        dt = times(t+1) - times(t);
        pos = [0 0];
        move = [S(t,1) S(t,2)] .* dt;

        NewLandmarks = zeros(0,2);
        for i = 1:19
            sensor = 49 + i;
            angle = ((i-10) / 9) * (0.5 * pi) * -1;

            if States(t+1,sensor) >= 100
                continue;
            end

            x = States(t+1, sensor) * cos(angle);% + new_pos(1);
            y = States(t+1, sensor) * sin(angle);% + new_pos(2);
            NewLandmarks = [NewLandmarks; [x y]]; 
        end

        % Localization
        % Search for right yawrate
        yawrate = 0;%-0.007;
        delta = 0.001;

        epsilon = 0.00001;
        [bestError, ResultingPoints] = computeProjectionError(Landmarks, move, NewLandmarks, yawrate);

        % Coordinate descent search
        while delta > epsilon
            yawrate = yawrate + delta;

            % Try with greater value
            [error, ResultingPoints] = computeProjectionError(Landmarks, move, NewLandmarks, yawrate);

            if error < bestError
                bestError = error;
                delta = delta * 1.1;
            else
                % If no success true smaller value
                yawrate = yawrate - (2 * delta);
                [error, ResultingPoints] = computeProjectionError(Landmarks, move, NewLandmarks, yawrate);
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

