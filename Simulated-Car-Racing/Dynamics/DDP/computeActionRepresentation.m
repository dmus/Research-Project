function U = computeActionRepresentation(Controls)
%COMPUTEACTIONREPRESENTATION Summary of this function goes here
%   Detailed explanation goes here
    U = zeros(size(Controls,1),2);
    speedControl = Controls(:,1) + -1 * Controls(:,2);
    
    U(:,1) = speedControl;
    U(:,2) = Controls(:,5);
end

