function cost = h(u)
%H Summary of this function goes here
%   Detailed explanation goes here
    %cost = sum(0.01 * u).^2;
    %cost = (u' * u) * 0.01;
    cost = (1 - u(1)) .* 2 + (-0.5 - u(2)) .^ 2;
end

