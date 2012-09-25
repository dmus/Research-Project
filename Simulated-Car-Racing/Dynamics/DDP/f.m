function s_next = f(s, u, dt, Map)
%F Summary of this function goes here
%   Detailed explanation goes here
    addpath('../Acceleration-Model');
    load('AccelerationLaggedModel.mat', 'model');

    % Make sure limits are respected in control inputs
    u(u > 1) = 1;
    u(u < -1) = -1;
    u(u(1:2) < 0) = 0;
    
    % Mapping to other input representation
    u(1) = u(1) + -1 * u(2);
    u(2) = u(3);
    u(3) = 1;
    
    % Compute accelerations
    s_temp = s';
    acc = zeros(3,1);
    acc(1:2) = model.Apos * mapStates(s_temp)' + model.Bpos * mapInputs(u',s_temp)';
    acc(3) = model.Arot * mapStates(s_temp)' + model.Brot * mapInputs(u',s_temp)';
                
    % Compute new velocities and new state features
    s_next(1:3,:) = s(1:3) + acc * dt;
    s_next(4:6,:) = phi(s(1:3), Map, s(4:6));
end

