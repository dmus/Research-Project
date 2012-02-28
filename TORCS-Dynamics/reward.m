function r = reward(state)
%REWARD Summary of this function goes here
%   Detailed explanation goes here
    
    % Penalize deviation from track axis and slow speed
    penalty = (state(2) / 7.5 - 1) ^2;
    penalty = penalty + (state(3)) ^2;
    %penalty = penalty + (((15000 / 3600) - state(4)) / (15000 / 3600)) .^2;
    
    r = -1 * sqrt(penalty);

%     r = 0;
%     
%     if state(4) > 0.1
%         r = 0.1;
%     end
%     
%     if state(2) < 1 || state(2) > 14
%         r = -1;
%     end
end

