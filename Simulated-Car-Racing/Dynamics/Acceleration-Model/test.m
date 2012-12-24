addpath('..', '../Yaw-Rate-Estimation', '../Trials');

S = load('Wheel-2_SimpleDriver.mat');

% Remove states and actions before start signal
States = S.States(S.States(:,2) >= 0,:);
Actions = S.Actions(S.States(:,2) >= 0,:);

% Compute time differences
times = computeDiscretizedTimes(States);
dt = times(2:end) - times(1:end-1);



% Build dataset
clear S;
S(:,1) = States(:,47) * 1000 / 3600;
S(:,2) = States(:,48) * 1000 / 3600;
[yawrates, uncertainty] = splineMethod(States);
S(:,3) = yawrates;

U(:,1) = Actions(:,1) + -1 * Actions(:,2);
U(:,2) = Actions(:,5);

% Exclude last examples, because we do not have an target value for that
% one
states = 5; % Number of states needed to predict next state
state_dim = 5; % State + input elements
X = zeros(size(times)-1, states * 5);
for i = 0:states-1
    first = (i*5) + 1;
    last = first + state_dim - 1;
    X(i+1:end,first:last) = [S(1:end-(i+1),:) U(1:end-(i+1),:)];
    %X(2:end,6:10) = [S(1:end-2,:) U(1:end-2,:)];
end

% X = [S(2:end-1,:) U(2:end-1,:) S(1:end-2,:) U(1:end-2,:)];

% Compute accelerations (targets)
y = S(2:end,3) - S(1:end-1,3);

% Delete rows with NaN
nans = (sum(isnan(X),2) > 0) | isnan(y);
X(nans,:) = [];
y(nans,:) = [];

% Number of examples
m = size(y,1);

num_test_examples = round(0.3 * m);
test_indices = randi(m, num_test_examples, 1);

X_test = X(test_indices,:);
X_training = X;
X_training(test_indices,:) = [];

y_test = y(test_indices,:);
y_training = y;
y_training(test_indices,:) = [];

save('data.mat', 'X_training', 'y_training', 'X_test', 'y_test');