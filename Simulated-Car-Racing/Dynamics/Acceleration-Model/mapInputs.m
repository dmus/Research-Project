function X = mapInputs(U)
%MAPINPUTS Summary of this function goes here
%   Detailed explanation goes here
    acc = U(:,1);
    acc(acc < 0) = 0;
    
    brake = -U(:,1);
    brake(brake < 0) = 0;

    X = [acc brake sqrt(acc) sqrt(brake) acc.^2 brake.^2 acc.^3 brake.^3 U(:,3)];
end

