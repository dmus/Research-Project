function r = reward(s, a)
%R Reward function.
%   Returns reward for state-action pair (S,A) 
    global Racetrack;

    r = 5;

    % Check if reached finish 
    if s(1) == 1
        r = 200;
        return;
    end
    
    % Check if going off-road
    new_pos = s(1:2) + s(3:4);
    if sum(new_pos' > size(Racetrack)) || ~Racetrack(new_pos(1), new_pos(2))
        r = 0;
    end
end

