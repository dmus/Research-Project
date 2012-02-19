function r = reward(state)
%REWARD Summary of this function goes here
%   Detailed explanation goes here
    r = state(1)^2; 

    if abs(state(2)) > 0.8 || abs(state(3)) > 0.5
        r = 0;
    end
    
    if abs(state(2)) > 1
        r = -1;
    end
end

