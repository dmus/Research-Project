function f = cosinusLaw(x, const, turnDirection)
%COSINUSLAW Summary of this function goes here
%   Detailed explanation goes here
    a = const(1);
    b = const(2);
    c = const(3);
    L = const(4);
    
    r = L / x;
    
    if strcmp('left', turnDirection)
        f = -b^2 + (r - a)^2 + (r - c)^2 - 2 * (r - a) * (r - c) * cos(x);
    elseif strcmp('right', turnDirection)
        f = -b^2 + (r + a)^2 + (r + c)^2 - 2 * (r + a) * (r + c) * cos(x);
    end
end

