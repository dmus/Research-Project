function r = reward(state)
%REWARD Summary of this function goes here
%   Detailed explanation goes here
    r = -1 * state(3)^2;
    
    if state(4) < 1
        r = -1000;
    end
end

