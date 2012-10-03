function cost = g(s, s_ref, alpha)
%G Cost function (negative reward)
%   Detailed explanation goes here

    deviationPenalty = 0;
    if nargin > 1
        deviationPenalty = sum((s - s_ref) .^2);
    end
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
    
    cost = -(speed^2) + s(5)^2;
    cost = (1 - alpha) * cost + alpha * deviationPenalty;
end

