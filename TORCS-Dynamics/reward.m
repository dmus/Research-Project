function r = reward(state)
%REWARD Summary of this function goes here
%   Detailed explanation goes here
    
    % Penalize deviation from track axis
    penalty = (state(2) / 7.5 - 1) ^2;
    penalty = penalty + (state(3) / pi) ^2;
    
    r = -1 * sqrt(penalty);
end

