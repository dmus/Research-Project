function F = phi(S)
%PHI Summary of this function goes here
%   Detailed explanation goes here
    F = zeros(size(S));

    % x-position
    F(:,1) = S(:,1);
    
    % y deviation from track axis
    F(:,2) = (S(:,2) / 7.5 - 1).^2;
    
    % deviation from track angle
    F(:,3) = S(:,3).^2;
    
    % speed along track axis
    F(:,4) = S(:,4) .* cos(S(:,3));
    
    % speed sideways
    F(:,5) = abs(S(:,4) .* sin(S(:,3)));
    
    % speed components of speed along transversal axis of the car
    %F(:,6) = abs(S(:,5) .* sin(S(:,3)));
    %F(:,7) = abs(S(:,5) .* cos(S(:,3)));
    
    % angular speed
    %F(:,6) = abs(S(:,5));
    
    % bias term
    F(:,6) = 1;
end

