function F = getActionFeatures(A)
%GETACTIONFEATURES Get state features for all states as row vectors in matrix S
%   Detailed explanation goes here
    F = zeros(size(A,1),3);
    
    F(:,1) = A(:,1);
    F(:,2) = A(:,2);
    F(:,3) = abs(A(:,2));
end

