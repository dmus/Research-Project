function cost = g(s)
%G Cost function (negative reward)
%   Detailed explanation goes here
    
    % Term for distance raced along track axis
    distance = s(11) - s(4);
    
    % Term for off-road
    offroad = (s(5) >= 1 || s(5) <= -1);
    
    cost = -1 * distance^2;
end

