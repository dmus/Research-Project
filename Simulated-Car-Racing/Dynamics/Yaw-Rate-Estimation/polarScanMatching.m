function yawRates = polarScanMatching(States)
%POLARSCANMATCHING Summary of this function goes here
%   Detailed explanation goes here
    % Compensation parameter 
    alpha = 0.815;

    S(:,1) = States(:,47) * 1000 / 3600;
    S(:,2) = States(:,48) * 1000 / 3600;
    yawRates = zeros(size(S,1),1);
    
    
    times = computeDiscretizedTimes(States);

    %t = 18139;%40;%536;

    %for t = 536:566
    for t = 536:536%size(States,1)
        fprintf('t = %0.3f\n', times(t));
        
        indices = 1:19;
        angles = transp(((indices - 10) / 9) * (0.5*pi) * -1);

        % Compute move
        dt = times(t) - times(t-1);
        translation = [S(t,1) S(t,2)] .* dt;

        % Initial pose of the current scan in the coordinate frame of the
        % reference scan. Bearings are expressent in current's scan
        % coordinate system
        C.position = translation;
        C.orientation = 0;
        C.scan = [flipud(States(t, indices + 49)') flipud(angles)];
        
        % The reference scan
        R.scan = [flipud(States(t-1, indices + 49)') flipud(angles)];
        
        % A. Scan Preprocessing
        % First ranges are segmented, to prevent interpolation between left
        % and right track edge
        C.points = [C.scan(:,1) .* cos(C.scan(:,2)) C.scan(:,1) .* sin(C.scan(:,2))];
        C.segments = segment(C.points, 10);
        
        R.points = [R.scan(:,1) .* cos(R.scan(:,2)) R.scan(:,1) .* sin(R.scan(:,2))];
        R.segments = segment(R.points, 10);
        
        % B. Scan Projection
        % Find out how the current scan would look like if it was taken
        % from the reference position
        C.projection(:,1) = sqrt((C.scan(:,1) .* cos(C.orientation + C.scan(:,2)) + C.position(1)) .^ 2 + (C.scan(:,1) .* sin(C.orientation + C.scan(:,2)) + C.position(2)) .^ 2);
        C.projection(:,2) = atan2(C.scan(:,1) .* sin(C.orientation + C.scan(:,2)) + C.position(2), C.scan(:,1) .* cos(C.orientation + C.scan(:,2)) + C.position(1));
        
        % First segment 1, reference scan bearings are calculated using
        % interpolation
        C.samples(:,2) = R.scan(R.segments(1,1):R.segments(1,2),2);
        
        % Interpolation step
        x = C.projection(R.segments(1,1):R.segments(1,2),2);
        y = C.projection(R.segments(1,1):R.segments(1,2),1);
        C.samples(:,1) = interp1(x, y, C.samples(:,2));
        
        % Now segment 2
        
        % C. Orientation Estimation
        yawrate = mean(C.samples(:,2) - C.projection(R.segments(1,1):R.segments(1,2),2));
        
        % Check if last step before finish
        if States(t+1,4) < States(t,4)-100
            yawrate = 0; % No better estimation available
        else
            
            yawrate = yawrate * alpha;
        end

        yawRates(t) = yawrate / dt;
    end
end

