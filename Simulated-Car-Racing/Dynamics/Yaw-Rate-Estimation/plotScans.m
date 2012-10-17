function plotScans(previousScan, newScan, translation, rotation)
%PLOTSCANS Summary of this function goes here
%   Detailed explanation goes here
    newScan = bsxfun(@plus,newScan,translation);

    if nargin > 3
        R = [cos(rotation) -sin(rotation); sin(rotation) cos(rotation)];
        newScan = (R * newScan')';
    end

    x = [0; translation(1); previousScan(:,1); newScan(:,1)];
    y = [0; translation(2); previousScan(:,2); newScan(:,2)];

    C = [.5 .5 .5;
         0 1 0;
         repmat([1 0 0], size(previousScan,1), 1);
         repmat([0 0 1], size(newScan,1), 1)];
    
    scatter(x, y, 9, C, 'fill');
    hold on;
    plot(previousScan(:,1), previousScan(:,2),'r-');
    hold off;
end

