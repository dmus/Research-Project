function [model, Trials] = updateDynamics(path)
%UPDATEDYNAMICS Summary of this function goes here
%   Detailed explanation goes here
    logs = dir(strcat(path, '/*.log'));
    num_trials = 0;
    
    for k = 1:numel(logs)
        reverse = fliplr(logs(k).name);
        name = fliplr(reverse(5:end));
        file = strcat(path, '/', name);
        if ~exist(strcat(file, '.mat'), 'file')
            convert(file);
        end
        
        % Load state-action trajectories
        load(strcat(file, '.mat'));
        
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

            Trials{num_trials + 1}.S = States(starts(i):stop,:);
            Trials{num_trials + 1}.A = Actions(starts(i):stop,:);
            
            num_trials = num_trials + 1;
        end
        
    end
    
    model = estimateDynamics(Trials);
end

