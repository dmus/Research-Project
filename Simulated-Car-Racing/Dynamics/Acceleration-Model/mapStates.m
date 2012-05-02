function X = mapStates(S)
%MAPSTATES Summary of this function goes here
%   Detailed explanation goes here
    
    X = [S(:,1) sqrt(S(:,1)) S(:,1).^2 S(:,2).*S(:,3) (S(:,2).*S(:,3)).^2];
end

