function X = mapStates(S)
%MAPSTATES Summary of this function goes here
%   Detailed explanation goes here
    
    X = [S(:,1) S(:,2) S(:,3) S(:,1).*S(:,3) abs(S(:,2)) abs(S(:,3))];
end

