% Test controller in our simulator, without the real simulator

H = 10;
driver = Controller(H);
map = [];

% Load Acceleration-model
addpath('../Acceleration-Model', '../Yaw-Rate-Estimation');
load('AccelerationLaggedModel.mat', 'model');

s = [0 0 0]';
time = 0;
dt = 0.02;

cost = 0;
for t = 1:H-1
    u = driver.control(s, time);
    cost = cost + g(s) + h(u);
    
    time = time + dt;
    s = f(s, u, dt, map, model);
end
cost = cost + g(s);
