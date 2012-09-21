function cost = g(s)
%G Cost function (negative reward)
%   Detailed explanation goes here
    
    cost = sum(s .^ 2);
end

