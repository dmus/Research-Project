function times = computeDiscretizedTimes(States)
%COMPUTEDISCRETIZEDTIMES Summary of this function goes here
%   Detailed explanation goes here
    times = States(:,2) - circshift(States(:,2),1);
    times(1) = 0;

    % Find resets for current lap time
    resets = find(times < 0);
    for i = 1:length(resets)
        ind = resets(i);
        times(ind) = States(ind,8) - States(ind-1,2) + States(ind,2);
    end

    times = cumsum(times);

end

