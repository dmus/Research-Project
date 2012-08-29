function T = buildMap(S, Sensors)
%BUILDMAP Summary of this function goes here
%   Detailed explanation goes here
    %times = computeDiscretizedTimes(Sensors);

    trackWidth = 12;
    
    len = size(S,1);
    
    Track = zeros(len,3);
    Left = zeros(len,2);
    Right = zeros(len,2);
    
    from = 1;%size(Track,1)-300;
    to = size(Track,1);
    %for t = 1:size(Track,1)
    for t = from:to
        angle = Sensors(t,1);
        orientation = S(t,3);
        alpha = orientation - angle;
        trackPos = Sensors(t,69) * (trackWidth / 2);
        
%         horizon = S(t,1:2) + [abs(trackPos) * cos(orientation) abs(trackPos) * sin(orientation)];
%         temp = horizon' - S(t,1:2)';
        
        if trackPos > 0 
            Track(t,1:2) = S(t,1:2) + [abs(trackPos) * cos(alpha - 0.5*pi) abs(trackPos) * sin(alpha - 0.5*pi)];
        else
            Track(t,1:2) = S(t,1:2) + [abs(trackPos) * cos(alpha + 0.5*pi) abs(trackPos) * sin(alpha + 0.5*pi)];
        end
        
        
        
%         R = [cos(beta) -sin(beta); sin(beta) cos(beta)];
%         Track(t,1:2) = (R * temp)' + S(t,1:2);
%         
        %trackPosLeft = 0.5 * trackWidth - trackPos;
        %trackPosRight = -0.5 * trackWidth - trackPos;
        
        Left(t,1:2) = Track(t,1:2) + [0.5*trackWidth * cos(alpha + 0.5*pi) 0.5*trackWidth * sin(alpha + 0.5*pi)];
        Right(t,1:2) = Track(t,1:2) + [0.5*trackWidth * cos(alpha - 0.5*pi) 0.5*trackWidth * sin(alpha - 0.5*pi)];
        
        %Track(t,1:2) = S(t,1:2) - [trackPos * sin(alpha) trackPos * cos(alpha)];
        %Left(t,1:2) = S(t,1:2) + [trackPosLeft * sin(alpha) trackPosLeft * cos(alpha)];
        %Right(t,1:2) = S(t,1:2) + [trackPosRight * sin(alpha) trackPosRight * cos(alpha)];
    end
    
    
    
    plotTrajectory(Track, 'b');
    hold on;
%     plotTrajectory(Left, 'black');
%     plotTrajectory(Right, 'black');
    plotTrajectory(S, 'r');
    
    plotTrajectory(Track(from:to,:), 'b');
    hold on;
    
    %plotTrajectory(Left(from:to,:), 'black');
    %plotTrajectory(Right(from:to,:), 'black');
    
    plotTrajectory(S(from:to,:), 'r');
    plot([Left(from:to,1) Right(from:to,1)]', [Left(from:to,2) Right(from:to,2)]','b-')
    plot([Track(from:to,1) S(from:to,1)]', [Track(from:to,2) S(from:to,2)]', 'b-');
    
    hold off;
    
    T = zeros(size(Track,1)-1,3);
    for t = 1:size(Track,1)-1
        distance = sqrt(sum((Track(t+1,:) - Track(t,:)) .^ 2));
        rotation = (S(t+1,3) - Sensors(t+1,1)) - (S(t,3) - Sensors(t,1));
        T(t,:) = [Sensors(t,4) distance rotation];
    end
    
end

