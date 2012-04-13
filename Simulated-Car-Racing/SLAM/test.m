%% Implementation of GraphSLAM
clear;
addpath('../Dynamics/Yaw-Rate-Estimation', '../Dynamics/Trials');
filename = 'track01_ExploreDriver.mat';

T = load(filename);
    
%% Remove states and actions before start signal
States = T.States(T.States(:,2) > 0,:);
Actions = T.Actions(T.States(:,2) > 0,:);

%% Compute longitudinal and lateral speeds in m/s
S(:,1) = States(:,47) * 1000 / 3600;
S(:,2) = States(:,48) * 1000 / 3600;
S(:,3) = estimateYawRate(States);

%S = S(1:30,:);

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