function handle = plotMap(Positions, LeftEdge, RightEdge)
%PLOTMAP Summary of this function goes here
%   Detailed explanation goes here
    x = [Positions(:,1); LeftEdge(:,1); RightEdge(:,1)];
    y = [Positions(:,2); LeftEdge(:,2); RightEdge(:,2)];

    C = [repmat([0 0 1], size(Positions,1), 1);
         repmat([1 0 0], size(LeftEdge,1), 1);
         repmat([1 0 0], size(RightEdge,1), 1)];
    
    handle = scatter(x, y, 2, C, 'fill');

end

