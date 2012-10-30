function [yawRates, LeftEdge, RightEdge, Positions] = splineMethod(States)
%FINDYAWRATE Most accurate method for yaw rate estimation.
%   Using splines and fminsearch in Cartesian coordinate system.
    
    % Compensation parameter 
    alpha = 0.9775;

    S(:,1) = States(:,47) * 1000 / 3600;
    S(:,2) = States(:,48) * 1000 / 3600;
    yawRates = zeros(size(S,1),1);
   
    times = computeDiscretizedTimes(States);

    t = 18139;%40;%536;
    %for t = 536:566
    
    for t = 2:size(States,1)
        fprintf('t = %0.3f\n', times(t));
        
        indices = 1:19;
        angles = transp(((indices - 10) / 9) * (0.5*pi) * -1);

        % Compute move
        dt = times(t) - times(t-1);
        translation = [.5*(S(t-1,1)+S(t,1)) .5*(S(t-1,2)+S(t,2))] .* dt;
        
        C.position = translation;
        C.orientation = 0;
        C.scan = [flipud(States(t, indices + 49)') flipud(angles)];
        
        % The reference scan
        R.scan = [flipud(States(t-1, indices + 49)') flipud(angles)];
        
        % A. Scan Preprocessing
        % First ranges are segmented, to prevent interpolation between left
        % and right track edge
        
        % Remove ranges > 30
        C.scan(C.scan(:,1) > 30,:) = [];
        R.scan(R.scan(:,1) > 30,:) = [];
        
        % TODO can this without computing points in x-y coordinates?
        C.points = [C.scan(:,1) .* cos(C.scan(:,2)) C.scan(:,1) .* sin(C.scan(:,2))];
        C.segments = segment(C.points, 10);
        
        R.points = [R.scan(:,1) .* cos(R.scan(:,2)) R.scan(:,1) .* sin(R.scan(:,2))];
        R.segments = segment(R.points, 10);
        
        yawRatesPerSegment = zeros(1,length(R.segments));
        for s = 1:length(R.segments)
            first = C.segments(s,1);
            last = C.segments(s,2);
            
            firstRef = R.segments(s,1);
            lastRef = R.segments(s,2);
        
            ppref = spline(R.points(firstRef:lastRef,1),R.points(firstRef:lastRef,2));
            f = @(x) scanError(C.points(first:last,:),ppref,C.position,x);
            yawRatesPerSegment(s) = fminsearch(f, 0);
        end
        yawrate = mean(yawRatesPerSegment);
        %yawrate = yawrate * alpha;
        
        if States(t,4) < States(t-1,4)-100
            yawrate = 0; % No better estimation available
        end

        yawRates(t-1) = (yawrate * alpha) / dt;
    end
end

