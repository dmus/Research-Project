function [Laps, start] = extractLaps(filename)
%EXTRACTLAP Laps extracted (information between start signal and first finish
%line passing is handled as separate lap)
%   Optional argument start returns the index of first sensor information
%   after start signal.
    Log = load(filename);
    
    % Remove states and actions before start signal
    start = find(Log.States(:,2) > 0, 1);
    
    States = Log.States(Log.States(:,2) > 0,:);
    Actions = Log.Actions(Log.States(:,2) > 0,:);

    % Separate into different trials for each lap
    % First, find starting points for each lap
    ind = find(States(:,4) < 2);
    starts = 1;
    for i = 1:length(ind)
        j = ind(i);
        if States(j - 1, 4) > States(j, 4)
            starts = [starts j];
        end
    end

    % Now make the trails
    num_trials = 0;
    
    for i = 1:length(starts)
        stop = size(States,1);
        if i < length(starts)
            stop = starts(i + 1) - 1;
        end

        S = States(starts(i):stop, :);
        A = Actions(starts(i):stop, :);
        
        Laps{num_trials + 1}.S = S;
        Laps{num_trials + 1}.A = A;
            
        num_trials = num_trials + 1;
    end 
    
end

