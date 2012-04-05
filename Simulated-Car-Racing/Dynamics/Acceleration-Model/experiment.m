addpath('../Yaw-Rate-Estimation');
addpath('../Trials');

trainingsrun = 'track01_MrRacer.mat';
testrun = trainingsrun;

% Number of steps to simulate
H = 50;

model = buildAccelerationOneStep(trainingsrun);
[velocityError, angularRateError] = testPerformance(model, H, testrun);