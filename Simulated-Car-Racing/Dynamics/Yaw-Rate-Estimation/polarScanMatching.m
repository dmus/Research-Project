function yawRates = polarScanMatching(States)
%POLARSCANMATCHING Summary of this function goes here
%   Detailed explanation goes here
    % Compensation parameter 
    alpha = 1;%0.815;

    S(:,1) = States(:,47) * 1000 / 3600;
    S(:,2) = States(:,48) * 1000 / 3600;
    yawRates = zeros(size(S,1),1);
    
    
    times = computeDiscretizedTimes(States);

    %t = 18139;%40;%536;

    for t = 15:20
    %for t = 2880:2881%size(States,1)
    %for t = 2:size(States,1)
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
        
        % Remove ranges > 30
        C.scan(C.scan(:,1) > 30,:) = [];
        R.scan(R.scan(:,1) > 30,:) = [];
        
        % TODO can this without computing points in x-y coordinates?
        C.points = [C.scan(:,1) .* cos(C.scan(:,2)) C.scan(:,1) .* sin(C.scan(:,2))];
        C.segments = segment(C.points, 10);
        
        R.points = [R.scan(:,1) .* cos(R.scan(:,2)) R.scan(:,1) .* sin(R.scan(:,2))];
        R.segments = segment(R.points, 10);
        
        % B. Scan Projection
        % Find out how the current scan would look like if it was taken
        % from the reference position
        ranges = sqrt((C.scan(:,1) .* cos(C.orientation + C.scan(:,2)) + C.position(1)) .^ 2 + (C.scan(:,1) .* sin(C.orientation + C.scan(:,2)) + C.position(2)) .^ 2);
        bearings = atan2(C.scan(:,1) .* sin(C.orientation + C.scan(:,2)) + C.position(2), C.scan(:,1) .* cos(C.orientation + C.scan(:,2)) + C.position(1));
        C.projection = [ranges bearings];
        
        % For both segments, reference scan bearings are calculated using
        % interpolation
        yawRatesPerSegment = zeros(1,length(R.segments));
        for s = 1:length(R.segments)
            first = R.segments(s,1);
            last = R.segments(s,2);
            bearings = R.scan(first:last,2);
        
            % Interpolation step
            x = C.projection(C.segments(s,1):C.segments(s,2),2);
            y = C.projection(C.segments(s,1):C.segments(s,2),1);
            C.samples = [interp1(x, y, bearings) bearings];
            C.samples(isnan(C.samples(:,1)),:) = [];
        
            % C. Orientation Estimation
            % Find shift of samples with respect to reference scan
            shifts = zeros(1,length(C.samples));
            for i = 1:length(C.samples)
                b = C.samples(i,:);
            
                % Up means r1 lies above r2, Down means r2 above r1
                optionsUp = find(R.scan(first:last-1,1) <= b(1) & R.scan(first+1:last,1) >= b(1));
                optionsDown = find(R.scan(first:last-1,1) >= b(1) & R.scan(first+1:last,1) <= b(1));
                options = [optionsUp optionsDown];
            
                if isempty(options)
                    shifts(i) = NaN;
                    continue; 
                end
            
                shiftOptions = zeros(size(options));
                for j = 1:length(options)
                    option = options(j);
                    r1 = R.scan(first - 1 + option,:);
                    r2 = R.scan(first + option,:);

                    dr = r2 - r1;
                    dbr1 = r1 - b;

                    ratio = abs(dbr1(1) / dr(1));
                    shiftOptions(j) = dbr1(2) + ratio * dr(2);
                end
            
                % Select smallest shift
                [~,index] = min(abs(shiftOptions));
                shifts(i) = shiftOptions(index);
            end
            
            % Remove greatest shift
            shifts(isnan(shifts)) = [];
            [~,index] = max(abs(shifts));
            shifts(index) = [];
            yawRatesPerSegment(s) = mean(shifts);
        end
        yawrate = mean(yawRatesPerSegment);
%         
%         figure(1);plot(R.scan(first:last,2),R.scan(first:last,1),'r-o',C.samples(:,2)+yawrate,C.samples(:,1),'b-o')
%         
%         figure(2);plotScans(R.points,C.points, translation, yawrate)
        
        
        % Check if last step before finish
        if States(t,4) < States(t-1,4)-100
            yawrate = 0; % No better estimate available
        else
            yawrate = yawrate * alpha;
        end

        yawRates(t-1) = yawrate / dt;
    end
end

