addpath('../Acceleration-Model', '../Yaw-Rate-Estimation', '../Trials');
file = 'Wheel-2_MrRacer.mat';

Laps = extractLaps(file);
S = computeStateRepresentation(Laps{2}.S);

save('S.mat', 'S');