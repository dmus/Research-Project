function plotStateTrajectory(S)
%PLOTSTATETRAJECTORY Summary of this function goes here
%   Detailed explanation goes here
    Colors = repmat([1 0 0], size(S,1), 1);
    scatter(S(:,1), S(:,2), 5, Colors, 'fill')
end

