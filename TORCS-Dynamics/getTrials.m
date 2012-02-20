function Trials = getTrials(filename)
%GETTRIALS Summary of this function goes here
%   Detailed explanation goes here
    T = load(filename);
    num_trials = 0;
    
    % Remove states and actions before start signal
    States = T.States(T.States(:,2) > 0,:);
    Actions = T.Actions(T.States(:,2) > 0,:);

    % Separate into different trials for each lap
    % First, find starting points for each lap
    ind = find(States(:,4) < 2);
    starts = [1];
    for i = 1:length(ind)
        j = ind(i);
        if States(j - 1, 4) > States(j, 4)
            starts = [starts j];
        end
    end

    num_features = 5;
    
    % Now make the trails
    for i = 1:length(starts)
        stop = size(States,1);
        if i < length(starts)
            stop = starts(i + 1) - 1;
        end

        S = States(starts(i):stop, :);
        A = Actions(starts(i):stop, :);
        
        % Feature selection for representing states
        Sn = zeros(size(S,1), num_features);
        Sn(:,1) = S(:,4) / 2057.56;
        Sn(:,2) = S(:,69);
        %Sn(S(:,69) > 0,2) = S(S(:,69) > 0,69); 
        %Sn(S(:,69) < 0,3) = S(S(:,69) < 0,69) * -1;
        Sn(:,3) = S(:,1) / pi;
        %Sn(S(:,1) > 0,4) = S(S(:,1) > 0,1);
        %Sn(S(:,1) < 0,5) = S(S(:,1) < 0,1) * -1;
        Sn(:,4) = (S(:,47) * 1000 / 3600) / 2057.56;
        Sn(:,5) = (S(:,48) * 1000 / 3600) ./ (S(:,50) + S(:,68));
        
        % Feature selection for representing actions
        An = zeros(size(A,1), 2);
        An(:,1) = A(:,1) - A(:,2);
        An(:,2) = A(:,5);
        
        Trials{num_trials + 1}.S = Sn;
        Trials{num_trials + 1}.A = An;
            
        num_trials = num_trials + 1;
    end    
end

