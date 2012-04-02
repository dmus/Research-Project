%% Implementation of Acceleration-One-Step
clear;

% Training run
filename = 'Trials/track01_MrRacer.mat';

T = load(filename);
    
%% Remove states and actions before start signal
States = T.States(T.States(:,2) >= 0,:);
Actions = T.Actions(T.States(:,2) >= 0,:);

times = States(:,2) - circshift(States(:,2),1);
times(1) = 0;

% Find resets for current lap time
resets = find(times < 0);
for i = 1:length(resets)
    ind = resets(i);
    times(ind) = States(ind,8) - States(ind-1,2) + States(ind,2);
end

times = cumsum(times);

%% Compute longitudinal and lateral speeds in m/s
S(:,1) = States(:,47) * 1000 / 3600;
S(:,2) = States(:,48) * 1000 / 3600;
S(:,3) = estimateYawRate(States);

%% Controls
U = [Actions(:, [1 2 5]) ones(size(Actions,1),1)];

%% Compute acclerations and store them in a matrix
Accelerations = zeros(size(S,1) - 1, size(S,2));

for t = 1:size(S,1) - 1
    dt = times(t+1) - times(t);
    
    % Rotate velocity at time t+1 back into the body frame at time t
    yawrate = S(t,3);
    R = [cos(-yawrate) -sin(-yawrate);sin(-yawrate) cos(-yawrate)];
    speedsBackRotated = R * S(t+1,1:2)';
    Accelerations(t,1:2) = (speedsBackRotated' - S(t,1:2)) / dt;
    
    % Also angular acceleration
    Accelerations(t,3) = S(t+1,3) - S(t,3);
end

Apos = zeros(2,3);
Bpos = zeros(2,4);
Arot = zeros(1,3);
Brot = zeros(1,4);

%% Apply linear regression

% Acceleration in x-direction
X = [S(1:end-1,:) U(1:end-1,[1 2 4])];
y = Accelerations(:,1);
theta = linearRegression(X, y);
Apos(1,:) = theta(1:3);
Bpos(1,[1 2 4]) = theta([4 5 6]);

% Acceleration in y-direction
X = [S(1:end-1,:) U(1:end-1,:)];
y = Accelerations(:,2);
theta = linearRegression(X, y);
Apos(2,:) = theta(1:3);
Bpos(2,:) = theta(4:7);

% Acceleration in angular speed
X = [S(1:end-1,:) U(1:end-1,:)];
y = Accelerations(:,3);
theta = linearRegression(X, y);
Arot(1,:) = theta(1:3);
Brot(1,:) = theta(4:7);

%% Simulate in current model
T = size(S,1);
H = 50;
Predictions = zeros(T, H, 3);

for t = 1:T
    for h = 1:H
        % Predict s at time t+h given s at time t
        s = S(t,:)';
        for i=1:h
            accelerations = zeros(3,1);
            accelerations(1:2) = Apos * s + Bpos * U(t+h,:)';
            accelerations(3) = Arot * s + Brot * U(t+h,:)';
            
            yawrate = s(3);
            R = [cos(yawrate) -sin(yawrate); sin(yawrate) cos(yawrate)];
            
            % Compute new state
            dt = times(t+h) - times(t+h-1);
            s(1:2) = R * (s(1:2) + accelerations(1:2) * dt);
            s(3) = s(3) + accelerations(3) * dt;
        end
        
        % Store prediction
        Predictions(t,h,:) = s;
    end
end

%% Solve 3 least-squares problem

% Construct dataset for number 1 and 2
X = zeros((T - H) * H, 6);
Y = zeros((T - H) * H, 2);

k = 1;
for t = 1:T - H
    for h = 1:H
        y = zeros(1,2);
        x = zeros(1,7);
        yaw = 0;
        for tau = 0:h-1 
            dt = times(t + tau + 1) - times(t + tau);
            
            R = [cos(-yawrate) -sin(-yawrate); sin(-yawrate) cos(-yawrate)];
            if tau > 0
                s = Predictions(t,tau,:);
            else
                s = S(t,:);
            end   
            
            
            x = x + ([s U(t+tau,:)] * R(1,1) + [s U(t+tau,:)] * R(1,2)) * dt;
            y = y + R * Accelerations(t+tau,1:2) * dt;
            
            yaw = yaw + s(3);
        end
        X(k,:) = x;
        Y(k,:) = y;
        k = k + 1;
    end
end

theta = linearRegression(X,Y(:,1));
A = zeros(size(Apos));
B = zeros(size(Apos));
A(1,:) = theta(1:3);
B(1,:) = theta(4:7);

%% Update A,B

% TODO linesearch to choose alpha
alpha = 0.1;

Apos = (1-alpha) * Apos + alpha * A;
Bpos = (1-alpha) * Bpos + alpha * B;

%% Check if converged
change = 0;
change = change + sqrt(sum((model{i+1}.Apos(:) - model{i}.Apos(:)) .^2));


