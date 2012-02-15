function Samples = sampleStates(Trials, number)
%SAMPLESTATES Summary of this function goes here
%   Detailed explanation goes here

    Samples = zeros(numel(Trials) * number, 5);
    row = 1;
    for i=1:numel(Trials)
        ind = ceil(rand(number, 1) * size(Trials{i}.S, 1));
        Samples(row:row + number - 1,:) = Trials{i}.S(ind, [4 69 1 47 48]);
        
        row = row + size(Trials{i}.S, 1);
    end
end

