function f = myfun(x)
%MYFUN Summary of this function goes here
%   Detailed explanation goes here
    g(1) = sin(6/(10*x));
    g(2) = 1 / (2 * x);

    f = g(1) - g(2);
end


