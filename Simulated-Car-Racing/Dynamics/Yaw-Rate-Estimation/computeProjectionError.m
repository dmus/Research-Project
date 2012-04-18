function [error, ResultingPoints] = computeProjectionError(LandmarksLeft, LandmarksRight, move, Ranges, angle)
%COMPUTEPROJECTIONERROR Summary of this function goes here
%   Detailed explanation goes here
    error = 0;
    ResultingPoints = zeros(size(Ranges));
    
    % Interpolate extra landmarks
    LandmarksLeftX = zeros(size(LandmarksLeft, 1) * 2 - 1, 2);
    LandmarksRightX = zeros(size(LandmarksRight, 1) * 2 - 1, 2);
    
    counter = 0;
    for i = 1:size(LandmarksLeft,1)-1
        LandmarksLeftX(counter+1,:) = LandmarksLeft(i,:);
        new = (LandmarksLeft(i,:) + LandmarksLeft(i+1,:)) / 2;
        LandmarksLeftX(counter+2,:) = new;
        
        counter = counter + 2;
    end
    LandmarksLeftX(end,:) = LandmarksLeft(end,:);
    
    % Smoothing, odd indices are fixed
    NewPath = LandmarksLeftX;
    weightSmooth = 0.1;
    gamma = 0.5 * weightSmooth;
    tolerance = 0.00001;
    
    change = tolerance;
    while change >= tolerance
        change = 0;
        for i=1:size(LandmarksLeft,1)
            if mod(i,2) == 0 && i > 2 
                aux = NewPath(i,:);
                
                NewPath(i,:) = NewPath(i,:) + weightSmooth * (NewPath(i-1,:) + NewPath(i+1,:) - 2*NewPath(i,:));
                
                NewPath(i,:) = NewPath(i,:) + gamma * (2 * NewPath(i-1,:) - NewPath(i-2,:));
                NewPath(i,:) = NewPath(i,:) + gamma * (2 * NewPath(i+1,:) - NewPath(i+2,:));
                
                change = change + sum(abs(aux - NewPath(i,:)));
            end
        end
    end
    
    LandmarksLeftX = NewPath;
    
    % Same for right
    counter = 0;
    for i = 1:size(LandmarksRight,1)-1
        LandmarksRightX(counter+1,:) = LandmarksRight(i,:);
        new = (LandmarksRight(i,:) + LandmarksRight(i+1,:)) / 2;
        LandmarksRightX(counter+2,:) = new;
        
        counter = counter + 2;
    end
    LandmarksRightX(end,:) = LandmarksRight(end,:);
    
    R = [cos(angle) -sin(angle); sin(angle) cos(angle)];

    index = 1;
    
    % Range finders at the left side
    for i = 1:size(Ranges,1)
        if abs(Ranges(i,2) - Ranges(1,2)) > 8
            break; % Continue with right side
        end
        
        Point = R * Ranges(i,:)' + move';
        ResultingPoints(i,:) = Point';
        
        while true
            dx = LandmarksLeft(index+1,1) - LandmarksLeft(index,1);
            dy = LandmarksLeft(index+1,2) - LandmarksLeft(index,2);
            normalizer = dx^2 + dy^2;
        
            Rx = Point(1) - LandmarksLeft(index,1);
            Ry = Point(2) - LandmarksLeft(index,2);
        
            u = (Rx * dx + Ry * dy) / normalizer;
            if u > 1
                index = index + 1;
                if index > size(LandmarksLeft,1)
                    break;
                end
            else
                break;
            end
        end
        
        if index > size(LandmarksLeft,1)
            break;
        end
        
        cte = (Ry * dx - Rx * dy) / normalizer;
        
        error = error + cte^2;
    end
    
    % Same for right side
    index = 1;
    for j = 1:size(Ranges,1)
        i = size(Ranges,1) + 1 - j;
        if abs(Ranges(i,2) - Ranges(end,2)) > 8
            break; % Left side
        end
        
        Point = R * Ranges(i,:)' + move';
        ResultingPoints(i,:) = Point';
        
        while true
            dx = LandmarksRight(index+1,1) - LandmarksRight(index,1);
            dy = LandmarksRight(index+1,2) - LandmarksRight(index,2);
            normalizer = dx^2 + dy^2;
        
            Rx = Point(1) - LandmarksRight(index,1);
            Ry = Point(2) - LandmarksRight(index,2);
        
            u = (Rx * dx + Ry * dy) / normalizer;
            if u > 1
                index = index + 1;
                if index > size(LandmarksRight,1)
                    break;
                end
            else
                break;
            end
        end
        
        if index > size(LandmarksRight,1)
            break;
        end
        
        cte = (Ry * dx - Rx * dy) / normalizer;
        
        error = error + cte^2;
    end
end

