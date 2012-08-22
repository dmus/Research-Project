trainingsrun = 'Wheel-2_MrRacer.mat';
testrun = 'E-Track-2_MrRacer.mat';

window = 5100:5300;%;1050:2000;

% Number of steps to simulate
T = load(trainingsrun);

% Remove states and actions before start signal
States = T.States(T.States(:,2) >= 0,:);
Actions = T.Actions(T.States(:,2) >= 0,:);

% Compute time differences
times = computeDiscretizedTimes(States);
times = times(window);

S_temp = S(window,:);
U_temp = U(window,:);

Accelerations = zeros(size(S_temp,1) - 1, size(S,2));

for t = 1:size(S_temp,1) - 1
    dt = times(t+1) - times(t);

    % Rotate velocity at time t+1 back into the body frame at time t
    yawrate = S_temp(t,3)*dt;
    R = [cos(-yawrate) -sin(-yawrate);sin(-yawrate) cos(-yawrate)];
    speedsBackRotated = R * S_temp(t+1,1:2)';
    Accelerations(t,1:2) = (speedsBackRotated' - S_temp(t,1:2)) / dt;

    % Also angular acceleration
    Accelerations(t,3) = (S(t+1,3) - S(t,3)) / dt;
end

% for t = 1:size(S_temp,1) - 1
%     dt = times(t+1) - times(t);
%     
%     angle = S_temp(t,3)*dt;
%     R = [cos(angle) -sin(angle);sin(angle) cos(angle)];
%     
%     px = R * (S_temp(t,1:2) + Accelerations(t,1:2)*dt)';
%     [px S_temp(t+1,1:2)']
% end

Sg = [zeros(size(S(window,:),1), 3) S(window,1:3)];
for t = 2:size(S_temp,1)
    prev = Sg(t-1,:);
    x = prev(1);
    y = prev(2);
    yaw = prev(3);
    speedX = prev(4);
    speedY = prev(5);
    yawRate = prev(6);
    
    dt = times(t) - times(t-1);
    
    x_new = x + (speedX * cos(yaw) + speedY * sin(yaw)) * dt;
    y_new = y + (speedX * sin(yaw) + speedY * cos(yaw)) * dt;
    yaw_new = yaw + (yawRate * dt);
    
    Sg(t, 1:3) = [x_new y_new yaw_new];
end

P = zeros(100, 6);
P(1,:) = Sg(1,:);
for t = 2:25
    prev = P(t-1,:);
    x = prev(1);
    y = prev(2);
    yaw = prev(3);
    speedX = prev(4);
    speedY = prev(5);
    yawRate = prev(6);
    
    dt = times(t) - times(t-1);
    
    acc = zeros(3,1);
    acc(1:2) = model.Apos * mapStates([speedX speedY yawRate])' + model.Bpos * mapInputs(U_temp(t-1,:),[speedX speedY yawRate])';
    acc(3) = model.Arot * mapStates([speedX speedY yawRate])' + model.Brot * mapInputs(U_temp(t-1,:),[speedX speedY yawRate])';
    
    x_new = x + (speedX * cos(yaw) + speedY * sin(yaw)) * dt;
    y_new = y + (speedX * sin(yaw) + speedY * cos(yaw)) * dt;
    yaw_new = yaw + (yawRate * dt);
    
    P(t, 1:3) = [x_new y_new yaw_new];
    
    a = yawRate * dt;
    R = [cos(a) -sin(a); sin(a) cos(a)];
    P(t, 4:5) = transp(R * ([speedX;speedY] + dt * acc(1:2)));
    P(t,6) = yawRate + acc(3)*dt;
end

C = [repmat([1 0 0], size(Sg,1), 1); repmat([0 0 1], size(P,1), 1)];
scatter([Sg(:,1); P(:,1)], [Sg(:,2); P(:,2)], 5, C, 'fill')