function reward = R(s, a)
%R Reward function.
%   Returns reward for state-action pair (S,A) 

    reward = 5;

    % Check if reached finish 
    if s(1) == 1
        reward = 200;
        return;
    end
    
    % Check if going off-road
    new_pos = s(1:2) + s(3:4);
    if ~Racetrack(new_pow(1), new_pos(2))
        reward = 0;
    end
end

