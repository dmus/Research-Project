% Discretize action space
velocities = [0.5 0.25 0 -0.5 -0.25];
steerings = [0.2 0 -0.2];
    
[x1,x2] = ndgrid(velocities, steerings);
Actions = [x1(:) x2(:)];

for i = 1:20
    [model, Trials] = updateDynamics('Trials');
    Samples = sampleStates(Trials, 100);

    %cost = computeCost(X,y(:,1),[model.A(1,:)'; model.B(1,:)']);

    % Approximate value function
    theta = findPolicy(model, Samples, Actions);

    % Save results
    save('dynamics.mat', 'model', 'theta', 'Actions');

    csvwrite('actions.csv', Actions);
    csvwrite('A.csv', model.A);
    csvwrite('B.csv', model.B);
    csvwrite('theta.csv', theta);

    command = 'java -classpath "../TORCS-Controller/classes;../TORCS-Controller/*" champ2011client.Client champ2011client.Schumacher verbose:on > %s';
    output = sprintf('Trials/trial%03d.log', i);
    system(sprintf(command, output));
    
    convert(sprintf('Trials/trial%03d', i));
    Trials = getTrials(sprintf('Trials/trial%03d.mat', i));
    
    durations = zeros(numel(Trials),1);
    for j = 1:numel(Trials)
        durations(j) = size(Trials{j}.S,1);
    end
    
    performance(i) = mean(durations)
    plot(performance)
end