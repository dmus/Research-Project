Map = buildMap(S,Laps{2}.S);
speedControl = Laps{2}.A(:,1) + -1 * Laps{2}.A(:,2);
U = [speedControl Laps{2}.A(:, 5) ones(size(Laps{2}.A,1),1)];
times = computeDiscretizedTimes(Laps{2}.S);


total_error = zeros(1,3);
for t = 1:size(U,1)-1
    dt = times(t+1) - times(t);
    
    s = S(t,:)';
    u = U(t,:)';
    s_new = f(s,u,dt);

    % Position of origin relative to track axis
    ref = Laps{2}.S(t,[4 1 69]);
    
    % Position + orientation relative to origin
    p = s(4:6) * dt;
    features = phi(p, Map, ref);
    
    % Compute squared error
    error = (Laps{2}.S(t+1,[4 1 69]) - features) .^ 2;
    
    total_error = total_error + error;
end
