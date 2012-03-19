clear;
filename = 'Trials/track01_MrRacer.mat';

T = load(filename);
num_trials = 0;
    
% Remove states and actions before start signal
States = T.States(T.States(:,2) > 0,:);
Actions = T.Actions(T.States(:,2) > 0,:);

S(:,1) = States(:,47) * 1000 / 3600;
S(:,2) = States(:,48) * 1000 / 3600;

range0 = States(:,50) * -1;
range1 = States(:,51) * -1;

R = [range0, zeros(length(range0), 1), range1 .* cos(pi / 18), range1 .* sin(pi / 18)];
B = R(:,[3 4]) - R(:, [1 2]);

C = atan(B(:,1) ./ B(:,2));
C = C - States(:,1);

range0 = States(:,67);
range1 = States(:,68);

R = [range0, zeros(length(range0), 1), range1 .* cos(pi / 18), range1 .* sin(pi / 18)];
B = R(:,[3 4]) - R(:, [1 2]);

C(:,2) = atan(B(:,1) ./ B(:,2));
C(:,2) = C(:,2) - States(:,1);

C(1:400,:)