% OLD

clear;
filename = 'Trials/track01_MrRacer.mat';

T = load(filename);
num_trials = 0;
    
% Remove states and actions before start signal
States = T.States(T.States(:,2) > 0,:);
Actions = T.Actions(T.States(:,2) > 0,:);

% Compute starts of new laps
ind = find(States(:,4) < 2);
starts = 1;
for i = 1:length(ind)
    j = ind(i);
    if States(j - 1, 4) > States(j, 4)
        starts = [starts j];
    end
end

% START ONLY FOR TESTING
States = States(starts(2):starts(3)-1,:);
% END ONLY FOR TESTING

S(:,1) = States(:,47) * 1000 / 3600;
S(:,2) = States(:,48) * 1000 / 3600;


%% Compute yaw rate
prevDistanceRaced = circshift(States(:,5),1);
b = States(:,5) - prevDistanceRaced;

% TODO starts
prevDistanceFromStart = circshift(States(:,4),1);
L = States(:,4) - prevDistanceFromStart;

a = ((States(:,69) + 1) ./ 2 .* 15) - 7.5;
c = circshift(a, 1);

angles = zeros(size(a, 1), 2);

for i = 2:length(b)
    const = [a(i) b(i) c(i) L(i)];
    [leftAngle, fval] = fsolve(@(x) cosinusLaw(x, const, 'left'), 0.1);
    [rightAngle, fval] = fsolve(@(x) cosinusLaw(x, const, 'right'), 0.1);
    angles(i, 1) = leftAngle;
    angles(i, 2) = rightAngle;
end

angles = angles .* -1;

yawRate = States(:,1) - circshift(States(:,1),1);
yawRate(1) = 0;

yawRate = angles(:,1);%yawRate + angles;
S(:,3) = yawRate;
%% Plot worldcoordinates
% Global coordinates
delta_t = 0.02;
Sg = [zeros(size(S,1), 3) S];
for i = 2:size(S,1)
    prev = Sg(i - 1,:);
    x = prev(1);
    y = prev(2);
    o = prev(3);
    speedX = prev(4);
    speedY = prev(5);
    yawRate = prev(6);
    
    xnew = x + (speedX * cos(o) + speedY * sin(o)) * delta_t;
    ynew = y + (speedX * sin(o) + speedY * cos(o)) * delta_t;
    onew = o + (yawRate) * delta_t;
    
    Sg(i, 1:3) = [xnew ynew onew];
end



scatter(Sg(:,1), Sg(:,2)*-1);
%scatter(Sg(starts(2):starts(3),1), Sg(starts(2):starts(3),2)*-1)