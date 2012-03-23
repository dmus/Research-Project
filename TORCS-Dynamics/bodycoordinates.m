clear;
filename = 'Trials/track01_MrRacer.mat';

T = load(filename);
num_trials = 0;
    
% Remove states and actions before start signal
States = T.States(T.States(:,2) > 0,:);
Actions = T.Actions(T.States(:,2) > 0,:);

S(:,1) = States(:,47) * 1000 / 3600;
S(:,2) = States(:,48) * 1000 / 3600;

% % Left
% range0 = States(:,50) * -1;
% range1 = States(:,51) * -1;
% 
% R = [range0, zeros(length(range0), 1), range1 .* cos(pi / 18), range1 .* sin(pi / 18)];
% B = R(:,[3 4]) - R(:, [1 2]);
% 
% C = atan(B(:,1) ./ B(:,2));
% C = C - States(:,1);
% 
% % Right
% range0 = States(:,67);
% range1 = States(:,68);
% 
% R = [range0, zeros(length(range0), 1), range1 .* cos(pi / 18), range1 .* sin(pi / 18)];
% B = R(:,[3 4]) - R(:, [1 2]);
% 
% C(:,2) = atan(B(:,1) ./ B(:,2));
% C(:,2) = C(:,2) - States(:,1);
% C = mean(C,2); % Angle track axis
% 
% C = circshift(C,1);
% C(1) = 0;
% D = circshift(States(:,1), 1); % Previous angles
% D(1) = 0;
% 
% S(:,3) = States(:,1) + C - D;
% Print
%C(1:400,:)

wheelRadius = 0.3179;
LeftSpeed = States(:,71) .* wheelRadius;
RightSpeed = States(:,70) .* wheelRadius;

%[LeftSpeed RightSpeed States(:,47) ./ 3.6]

L1 = LeftSpeed .* 0.02;
L2 = RightSpeed .* 0.02;

r2 = 1.94 ./ (1 - L2 ./ L1);
r1 = r2 - 1.94;

angle1 = L1 ./ r1;
angle2 = L2 ./ r2;

Rot = [angle1 .* 50 angle2 .* 50];

S(:,3) = mean(Rot,2);
U = [ones(size(Actions,1), 1) Actions(:, [1 2 5])];

Y = circshift(S,-1) - S; % St+1 - St
Y = Y(1:end-1,:); % Remove last row

X = [S U];
X = X(1:end-1,:); % Remove last row

% Compute initial model
model.A = zeros(3,3);
model.B = zeros(3,4);

for i = 1:size(Y,2)
    theta = linearRegression(X, Y(:,i));
    model.A(i,:) = theta(1:3);
    model.B(i,:) = theta(4:7);
end

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

ind = find(States(:,4) < 2);
starts = 1;
for i = 1:length(ind)
    j = ind(i);
    if States(j - 1, 4) > States(j, 4)
        starts = [starts j];
    end
end

%scatter(Sg(:,1), Sg(:,2)*-1);
scatter(Sg(starts(2):starts(3),1), Sg(starts(2):starts(3),2)*-1)