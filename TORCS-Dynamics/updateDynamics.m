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
        
        % Load state-action trajectories and convert to trials
        Temp = getTrials(strcat(file, '.mat'));
        Trials(num_trials + 1: num_trials + numel(Temp)) = Temp;
    end
    
    model = estimateDynamics(Trials);
end

