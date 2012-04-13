addpath('../Yaw-Rate-Estimation');
addpath('../Trials');

trainingsrun = 'Wheel-2_MrRacer.mat';
testrun = 'E-Track-2_MrRacer';

% Number of steps to simulate
H = 10;

[model, S, U, times] = buildAccelerationOneStep(trainingsrun);



[velocityError, angularRateError] = testPerformance(model, H, testrun);