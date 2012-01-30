function f = phi(ind)
%PHI Features for a state.
%   PHI(S) returns a feature vector for state with state index IND.
    global Racetrack size_statespace;

    s = zeros(4,1);
    [s(1), s(2), s(3), s(4)] = ind2sub(size_statespace, ind);
    
    % Features for (1) Finish reached, (2) Off-road and (3) Speed
    f = zeros(3,1);

    f(1) = s(1) == 1;
    f(2) = ~Racetrack(s(1), s(2));
    f(3) = max(s(3:4) / 2);
end

