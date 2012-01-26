function p = T(s0, a0, s1)
%T Summary of this function goes here
%   Detailed explanation goes here
    
    % racetrack (a), 1 means road, 0 = off-road
    Racetrack = ones(12,35);
    Racetrack(1:5, 1:32) = 0;
    Racetrack(10, 1:4) = 0;
    Racetrack(11, 1:8) = 0;
    Racetrack(12, 1:12) = 0;
    
    % actions example
    s = [6;1;0;0];
    a = [0;1];
    
    
    s(1:2) = s(1:2) + s(3:4);
    s(3:4) = s(3:4) + a;
end

