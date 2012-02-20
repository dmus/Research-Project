load('dynamics.mat');

steps = 500;

s = [0.9878 0.3337 0 0 0.0016 -0.0000 0.0000]';
s = repmat(s,1,size(Actions,1));


for step = 1:steps
    t1 = model.A * s;

    t2 = model.B * Actions';

    s = t1 + t2;
end