clear;
addpath('..', '../Yaw-Rate-Estimation', '../Trials');

trainingsrun = 'Wheel-2_MrRacer.mat';
testrun = 'E-Track-2_MrRacer.mat';

% Number of steps to simulate
H = 10;
window = 1:1000;
T = load(trainingsrun);

% Remove states and actions before start signal
States = T.States(T.States(:,2) >= 0,:);
Actions = T.Actions(T.States(:,2) >= 0,:);

% Compute time differences
times = computeDiscretizedTimes(States);

% Compute longitudinal and lateral speeds in m/s, angular speed in
% rad/s
% S(:,1) = States(:,47) * 1000 / 3600;
% S(:,2) = States(:,48) * 1000 / 3600;
% S(:,3) = findYawRates(States);
% 
% 
% % Controls, acceleration, braking and an additional 1
% speedControl = Actions(:,1) + -1 * Actions(:,2);
% U = [speedControl Actions(:, 5) ones(size(Actions,1),1)];
load('trainingsdata.mat');
[model, Accelerations] = buildAccelerationOneStep(S, U, times);

T = load(testrun);

% Remove states and actions before start signal
States = T.States(T.States(:,2) >= 0,:);
Actions = T.Actions(T.States(:,2) >= 0,:);

% Compute time differences
times = computeDiscretizedTimes(States);

% S_test(:,1) = States(:,47) * 1000 / 3600;
% S_test(:,2) = States(:,48) * 1000 / 3600;
% S_test(:,3) = findYawRates(States);
% 
% speedControl = Actions(:,1) + -1 * Actions(:,2);
% U_test = [speedControl Actions(:, 5) ones(size(Actions,1),1)];
load('testdata.mat');

[velocityError, angularRateError] = testPerformance(model, H, S_test, U_test, times);