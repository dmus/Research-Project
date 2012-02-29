function [model, Trials] = updateDynamics(path)
%UPDATEDYNAMICS Update dynamics model with all log files in directory PATH
%   [MODEL, TRIALS] = UPDATEDYNAMICS(PATH) reads all log files in the
%   directory accessed by path, converts file to matlab format if necessary
%   and estimates a dynamics model in MODEL. All used trials to estimate
%   the model are returned in TRIALS
    
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
        num_trials = numel(Trials);
    end
    
    model = estimateDynamics(Trials);
end

