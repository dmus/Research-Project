clear;
addpath('../Trials');
filename = 'Wheel-2_MrRacer.mat';

T = load(filename);
    
% Remove states and actions before start signal
States = T.States(T.States(:,2) > 0,:);
Actions = T.Actions(T.States(:,2) > 0,:);

States = States(1:5636,:);
Actions = Actions(1:5636,:);

times = computeDiscretizedTimes(States);
tic
[yawRates, LeftEdge, RightEdge, Positions] = findYawRates(States);
toc
S(:,1) = States(:,47) * 1000 / 3600;
S(:,2) = States(:,48) * 1000 / 3600;
S(:,3) = yawRates;

% Compute world coordinates
Sg = [zeros(size(S,1), 3) S(:,1:3)];
for t = 2:size(S,1)
    prev = Sg(t-1,:);
    x = prev(1);
    y = prev(2);
    yaw = prev(3);
    speedX = .5*(prev(4) + Sg(t,4));
    speedY = .5*(prev(5)+ Sg(t,5));
    yawRate = .5*(prev(6)+ Sg(t,6));
    
    dt = times(t) - times(t-1);
    
    x_new = x + (speedX * cos(yaw) + speedY * sin(yaw)) * dt;
    y_new = y + (speedX * sin(yaw) + speedY * cos(yaw)) * dt;
    yaw_new = yaw + (yawRate * dt);
    
    Sg(t, 1:3) = [x_new y_new yaw_new];
end

% Find starting points of new laps
ind = find(States(:,4) < 2);
starts = 1;
for i = 1:length(ind)
    j = ind(i);
    if States(j - 1, 4) > States(j, 4)
        starts = [starts j];
    end
end

plot(Sg(:,1), Sg(:,2), 'r');
%scatter(Sg(starts(2):starts(3),1), Sg(starts(2):starts(3),2)*-1);