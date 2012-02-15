[model, Trials] = updateDynamics('Demonstrations');
Samples = sampleStates(Trials, 100);

%cost = computeCost(X,y(:,1),[model.A(1,:)'; model.B(1,:)']);

% Approximate value function
theta = findPolicy(model, Samples);