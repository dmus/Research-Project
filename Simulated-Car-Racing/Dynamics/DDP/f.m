function s_next = f(s,u,dt)
%F Summary of this function goes here
%   Detailed explanation goes here
    addpath('../Acceleration-Model');
    load('AccelerationLaggedModel.mat', 'model');

    s_next = zeros(size(s));
    
    % Compute accelerations
    acc = zeros(3,1);
    acc(1:2) = model.Apos * mapStates(s')' + model.Bpos * mapInputs(u',s')';
    acc(3) = model.Arot * mapStates(s')' + model.Brot * mapInputs(u',s')';
                
    % Compute new velocities
    s_next(4:6) = s(4:6) + acc * dt;
    
    % Compute new position
    s_next(1:3) = s(1:3) + s(4:6) * dt;
end

