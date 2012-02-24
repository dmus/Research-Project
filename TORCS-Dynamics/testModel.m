%% Discretize action space
velocities = [0.5 0 -0.5];
steerings = [0.2 0 -0.2];
    
[x1,x2] = ndgrid(velocities, steerings);
Actions = [x1(:) x2(:)];

%% Create data set
Trials = getTrials('Trials/track01_simple.mat');

state_dimension = 6;
num_state_features = 8;
num_action_features = 2;

X = zeros(0,num_state_features + num_action_features);
y = zeros(0,state_dimension);

for i = 1:numel(Trials)
    S = Trials{i}.S;
    A = Trials{i}.A;
        
    X = [X; mapStates(S(1:end-1, :)) A(1:end-1, :)];
    y = [y; S(2:end, :)];
end

%% Divide in training and test set
m = size(X,1);
ind_test = ceil(rand(ceil(0.3 * m), 1) * m);
ind_training = setdiff([1:m]', ind_test); 

Xtest = X(ind_test,:);
ytest = y(ind_test,:);

X = X(ind_training,:);
y = y(ind_training,:);

%% Build model
model.A = zeros(state_dimension, num_state_features);
model.B = zeros(state_dimension, num_action_features);

% Compute model parameters with linear regression
for i = 1:size(y,2)
    theta = linearRegression(X,y(:,i));
    model.A(i,:) = theta(1:num_state_features);
    model.B(i,:) = theta(num_state_features + 1:end);
end

%% Compute error in linear regression model
errors = zeros(state_dimension,1);
for i = 1:state_dimension
    theta = [model.A(i,:) model.B(i,:)]';
    errors(i) = computeCost(X,y(:,i),theta);
end

errors

%% Simulate linear regression model
steps = 500;

% Start state
s = [0.0 10.0027 -0.0050 0 0 0]';
s = repmat(s,1,size(Actions,1));


for step = 1:steps
    t1 = model.A * mapStates(s')';

    t2 = model.B * Actions';

    s = t1 + t2;
end