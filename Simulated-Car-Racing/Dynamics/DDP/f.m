function s_next = f(s,u,dt)
%F Summary of this function goes here
%   Detailed explanation goes here
    
    model = load('AccelerationLaggedModel.mat');

    % Compute accelerations
    acc = zeros(3,1);
    acc(1:2) = model.Apos * mapStates(state')' + model.Bpos * mapInputs(U(t+tau,:),state')';
    acc(3) = model.Arot * mapStates(state')' + model.Brot * mapInputs(U(t+tau,:),state')';
                
    
    % Compute new velocities
    
    % Compute new position

    % New distance from track axis
    
    % New angle with respect to track axis
    
    % New distance to go along track axis
end

