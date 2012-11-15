addpath('..', '../Yaw-Rate-Estimation', '../Trials');

trainingsrun = 'Wheel-2_MrRacer.mat';
testrun = 'E-Track-2_MrRacer.mat';

% Number of steps to simulate
H = 25;
%window = 1:1000;
T = load(trainingsrun);

% Remove states and actions before start signal
States = T.States(T.States(:,2) >= 0,:);
Actions = T.Actions(T.States(:,2) >= 0,:);

% Compute time differences
times = computeDiscretizedTimes(States);



% Compute longitudinal and lateral speeds in m/s, angular speed in
% rad/s
S(:,1) = States(:,47) * 1000 / 3600;
S(:,2) = States(:,48) * 1000 / 3600;
S(:,3) = splineMethod(States);

% S = [S(:,1:3) zeros(size(S,1),3)];
% for t = 2:size(S,1)
%     prev = S(t-1,:);
%     x = prev(4);
%     y = prev(5);
%     yaw = prev(6);
%     speedX = prev(1);
%     speedY = prev(2);
%     yawRate = prev(3);
%     
%     dt = times(t) - times(t-1);
%     
%     x_new = x + (speedX * cos(yaw) + speedY * sin(yaw)) * dt;
%     y_new = y + (speedX * sin(yaw) + speedY * cos(yaw)) * dt;
%     yaw_new = yaw + (yawRate * dt);
%     
%     S(t, 4:6) = [x_new y_new yaw_new];
% end

% % Controls, acceleration, braking and an additional 1
speedControl = Actions(:,1) + -1 * Actions(:,2);
U = [speedControl Actions(:, 5) ones(size(Actions,1),1)];
% load('trainingsdata.mat');
%model = buildAccelerationOneStep(S, U, times);
models = buildAccelerationLagged(S, U, times, H);
model = models{length(models)};

T = load(testrun);

% Remove states and actions before start signal
States = T.States(T.States(:,2) >= 0,:);
Actions = T.Actions(T.States(:,2) >= 0,:);

% Compute time differences
times = computeDiscretizedTimes(States);

S_test(:,1) = States(:,47) * 1000 / 3600;
S_test(:,2) = States(:,48) * 1000 / 3600;
S_test(:,3) = splineMethod(States);

% S_test = [S_test(:,1:3) zeros(size(S_test,1),3)];
% for t = 2:size(S_test,1)
%     prev = S_test(t-1,:);
%     x = prev(4);
%     y = prev(5);
%     yaw = prev(6);
%     speedX = prev(1);
%     speedY = prev(2);
%     yawRate = prev(3);
%     
%     dt = times(t) - times(t-1);
%     
%     x_new = x + (speedX * cos(yaw) + speedY * sin(yaw)) * dt;
%     y_new = y + (speedX * sin(yaw) + speedY * cos(yaw)) * dt;
%     yaw_new = yaw + (yawRate * dt);
%     
%     S_test(t, 4:6) = [x_new y_new yaw_new];
% end

speedControl = Actions(:,1) + -1 * Actions(:,2);
U_test = [speedControl Actions(:, 5) ones(size(Actions,1),1)];
% load('testdata.mat');

error = testPerformance(model, H, S_test, U_test, times);