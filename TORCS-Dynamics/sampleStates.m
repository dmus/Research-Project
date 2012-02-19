function Samples = sampleStates(Trials, number)
%SAMPLESTATES Summary of this function goes here
%   Detailed explanation goes here

    num_features = size(Trials{1}.S,2);
    Samples = zeros(numel(Trials) * number, num_features);
    row = 1;
    for i=1:numel(Trials)
        ind = ceil(rand(number, 1) * size(Trials{i}.S, 1));
        Samples(row:row + number - 1,:) = Trials{i}.S(ind, :);
        
        row = row + size(Trials{i}.S, 1);
    end
end

