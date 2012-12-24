load('data.mat');



% Number of training examples
m = size(X_training,1);
previous_states = 2;

% Build trainingset
X_yawrate = zeros(m, 42);
for i = 0:previous_states
    j = i * 5;
    k = i * 6;
    % 6 features
    X_s = [X_training(:,[j+2 j+3 j+5])...
           X_training(:,j+1) .* X_training(:,j+3)...
           max(X_training(:,j+4), 0) .* X_training(:,j+3)...
           min(X_training(:,j+4), 0) .* X_training(:,j+3)];
    X_yawrate(:,k+1:k+size(X_s,2)) = X_s;
end

% Add delta's
%X_yawrate(:,31:36) = X_yawrate(:,1:6) - X_yawrate(:,7:12);
%X_yawrate(:,37:42) = X_yawrate(:,7:12) - X_yawrate(:,13:18);

% Build testset
X_yawrate2 = zeros(size(X_test,1), 42);
for i = 0:previous_states
    j = i * 5;
    k = i * 6;
    % 6 features
    X_s = [X_test(:,[j+2 j+3 j+5])...
           X_test(:,j+1) .* X_test(:,j+3)...
           max(X_test(:,j+4), 0) .* X_test(:,j+3)...
           min(X_test(:,j+4), 0) .* X_test(:,j+3)];
    X_yawrate2(:,k+1:k+size(X_s,2)) = X_s;
end

% Add delta's
%X_yawrate2(:,31:36) = X_yawrate2(:,1:6) - X_yawrate2(:,7:12);
%X_yawrate2(:,37:42) = X_yawrate2(:,7:12) - X_yawrate2(:,13:18);

theta = linearRegression(X_yawrate,y_training);

cost_training = computeCost(X_yawrate, y_training, theta)
cost_test = computeCost(X_yawrate2, y_test, theta)

h = X_yawrate * theta;
margin = 0.5 * y_training;
within_margin = (h > (y_training - margin)) & (h < (y_training + margin));
accuracy = sum(within_margin) / m;

% Gradient correct
gradients = sum(sign(h) == sign(y_training)) / m;

% net = newff(X_yawrate', y', [200]);
% net.trainParam.epochs = 50;
% net = train(net,X_yawrate',y');
% Y = sim(net,X_yawrate');
% plot(X_yawrate',Y,'o');