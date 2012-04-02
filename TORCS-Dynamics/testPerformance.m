function [ output_args ] = testPerformance(model, H, testrun)
%TESTPERFORMANCE Summary of this function goes here
%   Detailed explanation goes here
    T = load(testrun);
    
    % Remove states and actions before start signal
    States = T.States(T.States(:,2) >= 0,:);
    Actions = T.Actions(T.States(:,2) >= 0,:);

    % Compute states
    S(:,1) = States(:,47) * 1000 / 3600;
    S(:,2) = States(:,48) * 1000 / 3600;
    S(:,3) = estimateYawRate(States);

    % Controls
    U = [Actions(:, [1 2 5]) ones(size(Actions,1),1)];

    % Divide in non-overlapping windows
    
    for i = 1:size(S,1)
        
    end
    S(:,1)

end

