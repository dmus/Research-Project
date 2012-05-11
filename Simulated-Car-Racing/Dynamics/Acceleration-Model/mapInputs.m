function X = mapInputs(U, S)
%MAPINPUTS Summary of this function goes here
%   Detailed explanation goes here
    acc = U(:,1);
    acc(acc < 0) = 0;
    
    brake = -U(:,1);
    brake(brake < 0) = 0;

    steer = U(:,2);
    
    X = [acc brake steer acc.*S(:,2) brake.*S(:,2) acc.*S(:,3) brake.*S(:,3) U(:,3)];
end

