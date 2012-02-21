function F = phi(S)
%PHI Summary of this function goes here
%   Detailed explanation goes here
    F = zeros(size(S));

    F(:,1) = S(:,1).^2;
    F(:,2) = abs(S(:,2));
    F(:,3) = abs(S(:,3));
    F(:,4) = S(:,4);
    F(:,5) = abs(S(:,5));
    F(:,6) = abs(S(:,6));
end

