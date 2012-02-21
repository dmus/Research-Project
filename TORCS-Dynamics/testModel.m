%% Discretize action space
velocities = [0.5 0 -0.5];
steerings = [0.2 0 -0.2];
    
[x1,x2] = ndgrid(velocities, steerings);
Actions = [x1(:) x2(:)];

%% Create training set
Trials = getTrials('Trials/track01_mrracer.mat');

num_state_features = size(Trials{1}.S,2);
num_action_features = size(Trials{1}.A,2);

X = zeros(0,num_state_features + num_action_features);
y = zeros(0,num_state_features);

for i = 1:numel(Trials)
    S = Trials{i}.S;
    A = Trials{i}.A;
        
    X = [X; S(1:end-1, :) A(1:end-1, :)];
    y = [y; S(2:end, :)];
end

model.A = zeros(num_state_features);
model.B = zeros(num_state_features, 2);

% Compute model parameters with linear regression
for i = 1:num_state_features
    theta = linearRegression(X,y(:,i));
    model.A(i,:) = theta(1:num_state_features);
    model.B(i,:) = theta(num_state_features + 1:end);
end

%% Compute error in linear regression model
errors = zeros(num_state_features,1);
for i = 1:num_state_features
    theta = [model.A(i,:) model.B(i,:)]';
    errors(i) = computeCost(X,y(:,i),theta);
end

error

%% Simulate linear regression model
steps = 500;

% Start state
s = [0.9878 0.3337 -0.0016 0 0 0]';
s = repmat(s,1,size(Actions,1));


for step = 1:steps
    t1 = model.A * s;

    t2 = model.B * Actions';

    s = t1 + t2;
end