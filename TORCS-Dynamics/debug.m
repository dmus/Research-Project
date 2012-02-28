clc;
clear;
load('dynamics.mat');

Trials = getTrials('Trials/trial001.mat');
trial = Trials{2};

s = trial.S(1,:);

t1 = model.A * mapStates(s)';
t2 = model.B * Actions';

s_prime = repmat(t1, 1, 6) + t2;
for k = 1:4
    s_prime = model.A * mapStates(s_prime')' + model.B * Actions';
end

values = theta' * phi(s_prime')';
values

