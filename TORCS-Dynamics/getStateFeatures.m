function F = getStateFeatures(S)
%MAPSTATE Get state features for all states as row vectors in matrix S
%   Detailed explanation goes here
    F = zeros(size(S,1),7);
    
    % x-pos, y-pos and heading
    F(:,1:3) = S(:,1:3);
    
    % speed along longitudinal car-axis
    F(:,4) = S(:,4);
    
    % speed along x-axis and y-axis
    F(:,5) = S(:,4) .* cos(S(:,3));
    F(:,6) = S(:,4) .* sin(S(:,3));
    
    % lateral speed
%     F(:,7) = S(:,5);
%     F(:,8) = S(:,5) .* sin(S(:,3));
%     F(:,9) = S(:,5) .* cos(S(:,3));
    
    % angular speed
    F(:,7) = S(:,5);
    
end

