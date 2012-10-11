function cost = g(s)
%G Cost function (negative reward)
%   Detailed explanation goes here

    trackLength = 6205.46; 

    % Term for distance raced along track axis
    distance = s(11) - s(4);
    
    % Correction for when finish line is passed
    if abs(distance) > 100
        distance = trackLength - s(11) + s(4);
    end
    
    % Term for off-road
    offroad = (s(5) >= 1 || s(5) <= -1);
    
    % Speed
    speed = s(1);
    
    cost = -(distance^2) + 1000 * offroad;
    cost = distance^2;
end

