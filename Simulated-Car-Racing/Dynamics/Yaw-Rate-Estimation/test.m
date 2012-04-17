addpath('../Trials');
filename = 'track01_MrRacer.mat';

T = load(filename);
    
% Remove states and actions before start signal
States = T.States(T.States(:,2) > 0,:);
Actions = T.Actions(T.States(:,2) > 0,:);

%yawRates = findYawRates(States);
S(:,1) = States(:,47) * 1000 / 3600;
S(:,2) = States(:,48) * 1000 / 3600;
S(:,3) = yawRates_scan;



% Plot result
% C = repmat([1 0 0], size(Landmarks,1), 1);
% C = [C; [0 .3 .3]];
% C = [C; [.6 .6 0]];
% C = [C; repmat([0 0 1], size(ResultingPoints,1), 1)];
% 
% x = [Landmarks(:,2); pos(2); move(2); ResultingPoints(:,2)];
% y = [Landmarks(:,1); pos(1); move(1); ResultingPoints(:,1)];
% 
% scatter(x * -1, y, 10, C, 'fill');

% Compute world coordinates
Sg = [zeros(size(S,1), 3) S];
for t = 2:size(S,1)
    prev = Sg(t-1,:);
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
    
    Sg(t, 1:3) = [x_new y_new yaw_new];
end

ind = find(States(:,4) < 2);
starts = 1;
for i = 1:length(ind)
    j = ind(i);
    if States(j - 1, 4) > States(j, 4)
        starts = [starts j];
    end
end

scatter(Sg(1:2156,2)*-1, Sg(1:2156,1));
%scatter(Sg(starts(2):starts(3),1), Sg(starts(2):starts(3),2)*-1);