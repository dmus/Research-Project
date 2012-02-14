load('track01_mrracer.mat');

% Actions: velocity and steering control
A = Actions(:,1) + -1 * Actions(:,2);
A(:,2) = Actions(:,5);

% Remove states and actions before start signal
States = States(States(:,2) > 0,:);
Actions = Actions(States(:,2) > 0,:);

% Separate into different trials for each lap
% First, find starting points for each lap
ind = find(States(:,2) < 0.1);
starts = [];
for i = 1:length(ind)
    j = ind(i);
    if j == 1 || States(j - 1, 2) > States(j, 2)
        starts = [starts j];
    end
end

% Now make the trails
for i = 1:length(starts)
    stop = size(States,1);
    if i < length(starts)
        stop = starts(i + 1) - 1;
    end
    
    Trials{i}.S = States(starts(i):stop,:);
    Trials{i}.A = Actions(starts(i):stop,:);
end

model = estimateDynamics(Trials);


cost = computeCost(X,y(:,1),[model.A(1,:)'; model.B(1,:)']);

% Sample 1000 states from expert's trajectory
ind = ceil(rand(1000, 1) * size(States,1));
Samples = States(ind, [4 69 1 47 48]);

% Approximate value function
theta = findPolicy(model, Samples);