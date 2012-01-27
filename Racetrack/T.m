function p = T(s0, a0, s1)
%T Transition probability.
%   T computes the probability the agent will land in state S1 when taking
%   action A0 in state S0.
    
    p = 0;

    % racetrack (a), 1 means road, 0 = off-road
    Racetrack = ones(12,35);
    Racetrack(1:5, 1:32) = 0;
    Racetrack(10, 1:4) = 0;
    Racetrack(11, 1:8) = 0;
    Racetrack(12, 1:12) = 0;
    
    
    s_new(1:2, 1) = s0(1:2) + s0(3:4);
    
    % Check if not off-road
    if Racetrack(s_new(1), s_new(2))
        s0(1:2) = s_new;
    else
        s0(3:4) = 0; % Reset speed
        if s0 == s1
            p = 1;
        end
        
        return;
    end
    
    
    if s0(3:4) <= 1 % Speed is low
        if s0 == s1
            p = 0.1;
        else
            s0(3:4) = s0(3:4) + a0;
            if s0 == s1
                p = 0.9;
            end
        end
    else
        if s0 == s1
            p = 0.5;
        else
            s0(3:4) = s0(3:4) + a0;
            if s0 == s1
                p = 0.5;
            end
        end
    end
end

