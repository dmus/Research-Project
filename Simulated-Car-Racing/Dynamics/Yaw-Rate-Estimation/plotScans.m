function plotScans(previousScan, newScan, previousPos, newPos)
%PLOTSCANS Summary of this function goes here
%   Detailed explanation goes here
    x = [previousPos(1); newPos(1); previousScan(:,1); newScan(:,1)];
    y = [previousPos(2); newPos(2); previousScan(:,2); newScan(:,2)];

    C = [0 0 1;
         0 1 0;
         repmat([1 0 0], size(previousScan,1), 1);
         repmat([.5 .5 .5], size(newScan,1), 1)];
    
    scatter(y*-1, x, 9, C, 'fill');
end

