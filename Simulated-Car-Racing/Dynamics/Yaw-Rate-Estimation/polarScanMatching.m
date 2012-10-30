function yawRates = polarScanMatching(States)
%POLARSCANMATCHING Summary of this function goes here
%   Detailed explanation goes here
    % Compensation parameter 
    alpha = 1;%0.815;

    S(:,1) = States(:,47) * 1000 / 3600;
    S(:,2) = States(:,48) * 1000 / 3600;
    yawRates = zeros(size(S,1),1);
    
    times = computeDiscretizedTimes(States);

    sensorIndices = 1:19;
    angles = flipud(transp(((sensorIndices - 10) / 9) * (0.5*pi) * -1));
    
    %t = 18139;%40;%536;

    %for t = 15:20
    %for t = 2890:2891%size(States,1)
    for t = 2:size(States,1)
        fprintf('t = %0.3f\n', times(t));
        
        % Compute move
        dt = times(t) - times(t-1);
        translation = [.5*(S(t-1,1)+S(t,1)) .5*(S(t-1,2)+S(t,2))] .* dt;

        % Initial pose of the current scan in the coordinate frame of the
        % reference scan. Bearings are expressent in current's scan
        % coordinate system
        C.position = translation;
        C.orientation = 0;
        C.scan = [flipud(States(t, sensorIndices + 49)') angles];
        
        % The reference scan
        R.scan = [flipud(States(t-1, sensorIndices + 49)') angles];
        
        % A. Scan Preprocessing
        % First ranges are segmented, to prevent interpolation between left
        % and right track edge
        
%         segments = find(abs(C.scan(2:end,1) - C.scan(1:end-1,1)) > 3.4);
%         segments = [1 segments];
%         lengths = segments(2:end) - segments(1:end-1);
        
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
        
        %plotter = ScanPlotter(C,R);
        yawRatesPerSegment = zeros(1,length(R.segments));
        for s = 1:length(R.segments)
            first = C.segments(s,1);
            last = C.segments(s,2);
            
            firstRef = R.segments(s,1);
            lastRef = R.segments(s,2);
            
            resolution = 0.0001;
            precision = 0.001;
            pp = spline(R.scan(firstRef:lastRef,2),R.scan(firstRef:lastRef,1));
            
            xBlue = C.projection(first:last,2);
            yBlue = C.projection(first:last,1);
            
            if s > 1
                xBlue = flipud(xBlue);
                yBlue = flipud(yBlue);
            end
            
            indices = [false; abs(yBlue(2:end) - yBlue(1:end-1)) > 3.4];
            xBlue(indices,:) = [];
            yBlue(indices,:) = [];
            
            %yRedFun = @(x) ppval(pp,x);
            deltaX = 0;         % total delta
            deltaDeltaX = Inf;  % delta at each iteration
            yRedFunDer = fnder(pp);

            
            while abs(deltaDeltaX) > precision
                xRed = xBlue + deltaX;
                yRed = ppval(pp,xRed);
                yRedDer = fnval(yRedFunDer, xRed);

                deltaDeltaX = yRedDer \ (yBlue - yRed);

                deltaX = deltaX + deltaDeltaX;
            end
            yawRatesPerSegment(s) = deltaX;
            
%             xx = R.scan(firstRef,2):resolution:R.scan(lastRef,2);
%             yy = ppval(pp,xx);
%             
%             margin = 0.2 / resolution;
%             projection = C.projection(first:last,:);
%            
%             % Determine left or right shift, 0 = left, 1 = right
%             leftShift = mean(projection(:,2)) > mean(R.scan(firstRef:lastRef,2));
%             
%             xIndices = zeros(length(projection),1);
%             for i = 1:length(projection)
%                 % Compute the left and right bounds
%                 steps = ceil((projection(i,2) - R.scan(firstRef,2)) / resolution);
%                 
%                 if ~leftShift
%                     leftIndex = steps - 10;
%                     rightIndex = steps + margin;
%                 else
%                     leftIndex = steps - margin;
%                     rightIndex = steps + 10;
%                 end
%                 
%                 if leftIndex < 1
%                     leftIndex = 1;
%                 end
%                 if rightIndex > length(xx)
%                     rightIndex = length(xx);
%                 end
%                 
%                 delta = bsxfun(@minus, projection(i,1), yy(leftIndex:rightIndex));
%                 [d,temp] = min(abs(delta));
%                 
%                 if d > 0.02
%                     xIndices(i) = NaN;
%                 else
%                     xIndices(i) = leftIndex + temp - 1;
%                 end
%             end
%             
%             projection(isnan(xIndices),:) = [];
%             xIndices(isnan(xIndices)) = [];
%             refs = xx(xIndices)';
%             shifts = refs - projection(:,2);
%             yawRatesPerSegment(s) = mean(shifts);
           
%             Delta = bsxfun(@minus, projection(:,1), yy);
%             [~,xIndices] = min(abs(Delta),[],2);
%             refs = xx(xIndices)';
%             withinBounds = refs > Bounds(:,1) & refs < Bounds(:,2);
%             refs = refs(withinBounds);
%             projection = projection(withinBounds,:);
%             shifts = refs - projection(:,2);
%             yawRatesPerSegment(s) = mean(shifts);
            
%             pp = spline(R.scan(firstRef:lastRef,2),R.scan(firstRef:lastRef,1));
%             yRedFun = @(x) ppval(pp,x);
%             cost = @(deltaX) norm(C.projection(first:last,1) - arrayfun(yRedFun, C.projection(first:last,2) + deltaX));
%             yawRatesPerSegment(s) = fminsearch(cost, 0);
            
%             bearings = R.scan(first:last,2);
%         
%             % Interpolation step
%             x = C.projection(C.segments(s,1):C.segments(s,2),2);
%             y = C.projection(C.segments(s,1):C.segments(s,2),1);
%             C.samples = [interp1(x, y, bearings) bearings];
%             C.samples(isnan(C.samples(:,1)),:) = [];
%         
%             % Orientation Estimation
%             % Find shift of projection with respect to reference scan
%             shifts = zeros(1,last-first+1);
%             for i = first:last
%                 b = C.projection(i,:);
%             
%                 % Up means r1 lies above r2, Down means r2 above r1
%                 optionsUp = find(R.scan(firstRef:lastRef-1,1) <= b(1) & R.scan(firstRef+1:lastRef,1) >= b(1));
%                 optionsDown = find(R.scan(firstRef:lastRef-1,1) >= b(1) & R.scan(firstRef+1:lastRef,1) <= b(1));
%                 options = [optionsUp optionsDown];
%             
%                 if isempty(options)
%                     shifts(i-first+1) = NaN;
%                     continue; 
%                 end
%             
%                 shiftOptions = zeros(size(options));
%                 for j = 1:length(options)
%                     option = options(j);
%                     r1 = R.scan(firstRef - 1 + option,:);
%                     r2 = R.scan(firstRef + option,:);
% 
%                     dr = r2 - r1;
%                     dbr1 = r1 - b;
% 
%                     ratio = abs(dbr1(1) / dr(1));
%                     shiftOptions(j) = dbr1(2) + ratio * dr(2);
%                 end
%             
%                 % Select smallest shift
%                 [~,index] = min(abs(shiftOptions));
%                 shifts(i-first+1) = shiftOptions(index);
%             end
%             
%             shifts = zeros(1,length(C.samples));
%             for i = 1:length(C.samples)
%                 b = C.samples(i,:);
%             
%                 % Up means r1 lies above r2, Down means r2 above r1
%                 optionsUp = find(R.scan(first:last-1,1) <= b(1) & R.scan(first+1:last,1) >= b(1));
%                 optionsDown = find(R.scan(first:last-1,1) >= b(1) & R.scan(first+1:last,1) <= b(1));
%                 options = [optionsUp optionsDown];
%             
%                 if isempty(options)
%                     shifts(i) = NaN;
%                     continue; 
%                 end
%             
%                 shiftOptions = zeros(size(options));
%                 for j = 1:length(options)
%                     option = options(j);
%                     r1 = R.scan(first - 1 + option,:);
%                     r2 = R.scan(first + option,:);
% 
%                     dr = r2 - r1;
%                     dbr1 = r1 - b;
% 
%                     ratio = abs(dbr1(1) / dr(1));
%                     shiftOptions(j) = dbr1(2) + ratio * dr(2);
%                 end
%             
%                 % Select smallest shift
%                 [~,index] = min(abs(shiftOptions));
%                 shifts(i) = shiftOptions(index);
%             end
%             
%             % Remove greatest shift
%             shifts(isnan(shifts)) = [];
%             [~,index] = max(abs(shifts));
%             shifts(index) = [];
%             yawRatesPerSegment(s) = mean(shifts);
        end
        yawrate = mean(yawRatesPerSegment);
        
        
%         
%         figure(1);plot(R.scan(firstRef:lastRef,2),R.scan(firstRef:lastRef,1),'r-o',C.projection(first:last,2),C.projection(first:last,1),'b-o')
%         
%         figure(2);plotScans(C,R)
%         
        
        % Check if last step before finish
        if States(t,4) < States(t-1,4)-100
            yawrate = 0; % No better estimate available
        else
            yawrate = yawrate * alpha;
        end

        yawRates(t-1) = yawrate / dt;
    end
end

