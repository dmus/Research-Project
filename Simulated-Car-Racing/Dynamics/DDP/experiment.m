Map = buildMap(S,Laps{2}.S);
speedControl = Laps{2}.A(:,1) + -1 * Laps{2}.A(:,2);
U = [speedControl Laps{2}.A(:, 5) ones(size(Laps{2}.A,1),1)];
times = computeDiscretizedTimes(Laps{2}.S);

for t = 1:size(U,1)-1
    dt = times(t+1) - times(t);
    
    s = S(t,:)';
    u = U(t,:)';
    s_new = f(s,u,dt);

    ref = Laps{2}.S(t,[4 1 69]);
    features = phi(s_new, Map, ref);
    
    % Compute squared error
    error = (Laps{2}.S(t+1,[4 1 69]) - features) .^ 2;
end