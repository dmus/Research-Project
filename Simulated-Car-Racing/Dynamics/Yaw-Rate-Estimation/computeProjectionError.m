function [error, ResultingPoints] = computeProjectionError(Landmarks, move, Ranges, angle)
%COMPUTEPROJECTIONERROR Summary of this function goes here
%   Detailed explanation goes here
    error = 0;
    ResultingPoints = zeros(size(Ranges));
    
    R = [cos(angle) -sin(angle); sin(angle) cos(angle)];

    index = 1;
    
    % Range finders at the left side
    for i = 1:size(Ranges,1)
        if abs(Ranges(i,2) - Ranges(1,2)) > 10
            break; % Continue with right side
        end
        
        Point = R * Ranges(i,:)' + move';
        ResultingPoints(i,:) = Point';
        
        while true
            dx = Landmarks(index+1,1) - Landmarks(index,1);
            dy = Landmarks(index+1,2) - Landmarks(index,2);
            normalizer = dx^2 + dy^2;
        
            Rx = Point(1) - Landmarks(index,1);
            Ry = Point(2) - Landmarks(index,2);
        
            u = (Rx * dx + Ry * dy) / normalizer;
            if u > 1
                index = index + 1;
            else
                break;
            end
        end
        
        if index > size(Landmarks,1)
            break;
        end
        
        cte = (Ry * dx - Rx * dy) / normalizer;
        
        error = error + cte^2;
    end
    
    % Same for right side
    index = size(Landmarks,1);
    for j = 1:size(Ranges,1)
        i = size(Ranges,1) + 1 - j;
        if abs(Ranges(i,2) - Ranges(end,2)) > 10
            break; % Left side
        end
        
        Point = R * Ranges(i,:)' + move';
        ResultingPoints(i,:) = Point';
        
        while true
            dx = Landmarks(index-1,1) - Landmarks(index,1);
            dy = Landmarks(index-1,2) - Landmarks(index,2);
            normalizer = dx^2 + dy^2;
        
            Rx = Point(1) - Landmarks(index,1);
            Ry = Point(2) - Landmarks(index,2);
        
            u = (Rx * dx + Ry * dy) / normalizer;
            if u > 1
                index = index - 1;
            else
                break;
            end
        end
        
        if index < 1
            break;
        end
        
        cte = (Ry * dx - Rx * dy) / normalizer;
        
        error = error + cte^2;
    end
end

