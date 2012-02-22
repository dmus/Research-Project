function F = mapStates(S)
%MAPSTATE Summary of this function goes here
%   Detailed explanation goes here
    F = zeros(size(S,1),8);
    
    F(:,1:3) = S(:,1:3);
    F(:,8) = S(:,6);

    F(:,4) = S(:,4) .* cos(S(:,3));
    F(:,5) = S(:,4) .* sin(S(:,3));
    
    F(:,6) = S(:,5) .* cos(S(:,3));
    F(:,7) = S(:,5) .* sin(S(:,3));
end

