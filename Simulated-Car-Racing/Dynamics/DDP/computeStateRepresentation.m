function S = computeStateRepresentation(Sensors)
%COMPUTESTATEREPRESENTATION Summary of this function goes here
%   Detailed explanation goes here
    S = zeros(size(Sensors,1),6);
    
    S(:,4) = Sensors(:,47) * 1000 / 3600;
    S(:,5) = Sensors(:,48) * 1000 / 3600;
    S(:,6) = findYawRates(Sensors);
    
    times = computeDiscretizedTimes(Sensors);
    
    for t = 2:size(S,1)
        prev = S(t-1,:);
        x = prev(1);
        y = prev(2);
        yaw = prev(3);
        speedX = prev(4);
        speedY = prev(5);
        yawRate = prev(6);
    
        dt = times(t) - times(t-1);
    
        x_new = x + (speedX * cos(yaw) + speedY * sin(yaw)) * dt;
        y_new = y + (speedX * sin(yaw) + speedY * cos(yaw)) * dt;
        yaw_new = yaw + (yawRate * dt);
    
        S(t, 1:3) = [x_new y_new yaw_new];
    end
end

