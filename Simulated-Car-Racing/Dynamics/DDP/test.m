addpath('../Trials', '../Yaw-Rate-Estimation', '../Acceleration-Model');
load('Wheel-2_SimpleDriver.mat');

% Remove states and actions before start signal
States = States(States(:,2) >= 0,:);
Actions = Actions(States(:,2) >= 0,:);

% Remove states and actions before finish line
ind = find(States(:,4) < 10 & States(:,2) < 10 & States(:,5) < 50);
States = States(ind(1):end,:);
Actions = Actions(ind(1):end,:);

times = computeDiscretizedTimes(States);

% Compute longitudinal and lateral speeds in m/s, angular speed in
% rad/s
% S(:,1) = States(:,47) * 1000 / 3600;
% S(:,2) = States(:,48) * 1000 / 3600;
% S(:,3) = findYawRates(States);
%
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
load('S.mat');

start = 1;
stop = length(S)-1;

% % Determine track axis
% A = zeros(size(S,1), 3); % [index, length, rotation]
% %for t = 1:size(S,1)-1
% for t = start:stop
%     dt = times(t+1) - times(t);
%     
%     deviation = States(t,69) * 12;
%     alpha = States(t,1);
%     
%     deviation_new = States(t+1,69) * 12;
%     alpha_new = States(t+1,1);
%     
%     move = S(t,1:2)' * dt;
%     pos = [0 deviation]';
%     pos_new = pos + move;
%     
%     R = [cos(alpha) -sin(alpha); sin(alpha) cos(alpha)];
%     
%     pos_temp = pos_new - pos;
%     pos_temp = R * pos_temp;
%     pos_new = pos_temp + pos;
%     
%     yaw_new = alpha + S(t,3) * dt;
%     beta = (0.5*pi) - ((0.5*pi) - (yaw_new - alpha_new));
%     
%     xplus = sin(beta) * deviation_new;
%     yplus = cos(beta) * deviation_new;
%     
%     landmark_new = pos_new - [xplus; yplus];
%     
%     distance = sqrt(sum(landmark_new .^ 2));
%     rotation = yaw_new - alpha_new;
%     %rotation = atan(landmark_new(1) / landmark_new(2));
%     
%     A(t,:) = [States(t,4) distance rotation];
% end

T = zeros(size(S,1),3);
R = zeros(size(T,1),3);
%for t=1:size(T,1)-1
for t=start:stop
    dt = times(t+1) - times(t);
    orientation = T(t,3);
    move = sqrt(sum((S(t,1:2) * dt) .^ 2));
    T(t+1,1:2) = T(t,1:2) + [cos(orientation) * move sin(orientation) * move];
    T(t+1,3) = T(t,3) + S(t,3) * dt;
    
    if States(t,69) > 0
        angle = States(t,1) + 0.5 * pi;
        Rot = [cos(-angle) -sin(-angle); sin(-angle) cos(-angle)];
    else
        angle = States(t,1) - 0.5 * pi;
        Rot = [cos(angle) -sin(angle); sin(angle) cos(angle)];
    end
    
    temp = T(t,1:2) + [States(t,69)*12 * cos(orientation) States(t,69)*12 * sin(orientation)];
    temp = Rot * (temp - T(t,1:2))';
    R(t,1:2) = temp' + T(t,1:2);
    
    %R(t,:) = T(t,1:2) + [States(t,69) * 12 * cos(rotation) States(t,69) * 12 * sin(rotation)];
end

for t=start:stop
    len = sqrt(sum((R(t+1,:) - R(t,:)) .^ 2));
    angle1 = T(t,3) - States(t,1);
    angle2 = T(t+1,3) - States(t+1,1);
    A(t,:) = [States(t,4) len angle2 - angle1];
end

P = zeros(size(A,1)+1,3);
P(1,:) = [R(1,1:2) States(1,1)];
for t=1:size(A,1)
    orientation = P(t,3);
    P(t+1,1:2) = P(t,1:2) + [cos(orientation) * A(t,2) sin(orientation) * A(t,2)];
    P(t+1,3) = P(t,3) + A(t,3);
end

scatter(P(:,1),P(:,2),1,'b');
hold on;
scatter(T(:,1),T(:,2),1,'r');
hold on;
scatter(R(:,1),R(:,2),1,'g');