function plotScans(previousScan, newScan, previousPos, newPos)
%PLOTSCANS Summary of this function goes here
%   Detailed explanation goes here
    x = [previousPos(1); newPos(1); previousScan(:,1); newScan(:,1)];
    y = [previousPos(2); newPos(2); previousScan(:,2); newScan(:,2)];

    C = [0 0 1;
         0 1 0;
         repmat([1 0 0], size(previousScan,1), 1);
         repmat([.5 .5 .5], size(newScan,1), 1)];
    
    scatter(x, y, 9, C, 'fill');
    
    % plot(previousScan(1:8,2)*-1,previousScan(1:8,1),'-rs',previousScan(9:end,2)*-1,previousScan(9:end,1),'-rs', newScan(1:8,2)*-1,newScan(1:8,1),'bo',newScan(9:end,2)*-1,newScan(9:end,1),'bo',previousPos(2)*-1,previousPos(1),'go',newPos(2)*-1,newPos(1),'go','Markersize',5,'MarkerFaceColor','b')
end

