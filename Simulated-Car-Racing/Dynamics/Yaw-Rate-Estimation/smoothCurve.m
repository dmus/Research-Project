function S = smoothCurve(P, fixed, weight)
%SMOOTHCURVE Summary of this function goes here
%   Detailed explanation goes here
    S = P;
    gamma = 0.5 * weight;
    tolerance = 0.00001;

    change = tolerance;
    while change >= tolerance
        change = 0;
        for i = 1:size(P,1)
            if mod(i,2) == 0 && i > 2 && i < size(P,1) - 1
                aux = S(i,:);

                S(i,:) = S(i,:) + weight * (S(i-1,:) + S(i+1,:) - 2*S(i,:));

                S(i,:) = S(i,:) + gamma * (2 * S(i-1,:) - S(i-2,:) - S(i,:));
                S(i,:) = S(i,:) + gamma * (2 * S(i+1,:) - S(i+2,:) - S(i,:));

                change = change + sum(abs(aux - S(i,:)));
            end
        end
    end

end

