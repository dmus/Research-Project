function [error, ResultingPoints] = computeProjectionError(LandmarksLeft, LandmarksRight, move, Ranges, angle)
%COMPUTEPROJECTIONERROR Summary of this function goes here
%   Detailed explanation goes here
    error = 0;
    
    R = [cos(angle) -sin(angle); sin(angle) cos(angle)];
    ResultingPoints = (R * bsxfun(@plus, Ranges, move)')';
    
    % Range finders at the left side
    index = 1;
    for i = 1:size(Ranges,1)
        if abs(Ranges(i,2) - Ranges(1,2)) > 8
            break; % Continue with right side
        end
        
        Point = ResultingPoints(i,:)';

        while true
            dx = LandmarksLeft(index+1,1) - LandmarksLeft(index,1);
            dy = LandmarksLeft(index+1,2) - LandmarksLeft(index,2);
            normalizer = dx^2 + dy^2;
        
            Rx = Point(1) - LandmarksLeft(index,1);
            Ry = Point(2) - LandmarksLeft(index,2);
        
            u = (Rx * dx + Ry * dy) / normalizer;
            if u > 1
                index = index + 1;
                if index >= size(LandmarksLeft,1)
                    break;
                end
            else
                break;
            end
        end
        
        if index >= size(LandmarksLeft,1)
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
        
        if size(LandmarksRight,1) == 0
            disp('Stop');
        end
        
        Point = ResultingPoints(i,:)';
        
        while true
            dx = LandmarksRight(index+1,1) - LandmarksRight(index,1);
            dy = LandmarksRight(index+1,2) - LandmarksRight(index,2);
            normalizer = dx^2 + dy^2;
        
            Rx = Point(1) - LandmarksRight(index,1);
            Ry = Point(2) - LandmarksRight(index,2);
        
            u = (Rx * dx + Ry * dy) / normalizer;
            if u > 1
                index = index + 1;
                if index >= size(LandmarksRight,1)
                    break;
                end
            else
                break;
            end
        end
        
        if index >= size(LandmarksRight,1)
            break;
        end
        
        cte = (Ry * dx - Rx * dy) / normalizer;
        
        error = error + cte^2;
    end
end

