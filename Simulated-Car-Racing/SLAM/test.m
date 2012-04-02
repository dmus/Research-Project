%% Implementation of GraphSLAM
clear;
filename = '../Trials/track01_MrRacer.mat';

T = load(filename);
    
%% Remove states and actions before start signal
States = T.States(T.States(:,2) > 0,:);
Actions = T.Actions(T.States(:,2) > 0,:);

%% Compute longitudinal and lateral speeds in m/s
S(:,1) = States(:,47) * 1000 / 3600;
S(:,2) = States(:,48) * 1000 / 3600;

%% Estimate yaw rate
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
S = S(1:30,:);
%% Compute global coordinates
% TODO project onto track-axis
delta_t = 0.02;
G = zeros(size(S,1), 3);
for i = 2:size(S,1)
    prev = G(i - 1,:);
    x = prev(1);
    y = prev(2);
    o = prev(3);
    
    prev = S(i - 1,:);
    speedX = prev(1);
    speedY = prev(2);
    yawRate = prev(3);
    
    xnew = x + (speedX * cos(o) + speedY * sin(o)) * delta_t;
    ynew = y + (speedX * sin(o) + speedY * cos(o)) * delta_t;
    onew = o + (yawRate) * delta_t;
    
    G(i, :) = [xnew ynew onew];
end

%% Build omega and xi matrices
Omega = sparse([],[],[],size(S,1) * 3,size(S,1) * 3,size(G,1) * 9);
xi = zeros(size(S,1) * 3, 1);

%% Enter constraints

% Start at (0,0,0) constraint
Omega(1:3,1) = 1;

% Odometry constraints with lower confidence
confidence = 0.5;
for i = 2:size(G,1)
    delta = G(i,:) - G(i-1,:);
    
    indicesPrevious = ones(3,1) * 3 * (i-2) + [1 2 3]';
    indicesCurrent = ones(3,1) * 3 * (i-1) + [1 2 3]';
    
    % Diagonal elements
    Omega(indicesPrevious, indicesPrevious) = Omega(indicesPrevious, indicesPrevious) + confidence;
    Omega(indicesCurrent, indicesCurrent) = Omega(indicesCurrent, indicesCurrent) + confidence;
    
    % Off-diagonal
    Omega(indicesPrevious, indicesCurrent) = Omega(indicesPrevious, indicesCurrent) - confidence;
    Omega(indicesCurrent, indicesPrevious) = Omega(indicesCurrent, indicesPrevious) - confidence;
    
    xi(indicesPrevious) = xi(indicesPrevious) + delta'; 
    xi(indicesCurrent) = xi(indicesCurrent) - delta';
end

% TODO more constraints

%% Compute estimation for path
mu = pinv(Omega) * mu;