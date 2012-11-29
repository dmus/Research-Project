function s_next = f(s, u, dt, Map, model)
%F Predicts next state.
%   Detailed explanation goes here
    s_next = s;
    
    % Make sure limits are respected in control inputs
    u(u > 1) = 1;
    u(u < -1) = -1;
    
    % Mapping to other input representation
    u = [u(1:2); 1];
    
    % Compute accelerations
    s_temp = s';
    acc = zeros(3,1);
    acc(1:2) = model.Apos * mapStates(s_temp)' + model.Bpos * mapInputs(u',s_temp)';
    acc(3) = model.Arot * mapStates(s_temp)' + model.Brot * mapInputs(u',s_temp)';
                
    % Compute new velocities and new state features
    angle = s(3) * dt;
    R = [cos(angle) -sin(angle); sin(angle) cos(angle)];
    s_next(1:2) = R * (s(1:2) + acc(1:2) * dt); 
    s_next(3) = s(3) + acc(3) * dt;
    
%     s_next(4:6) = phi(s(1:3), Map, s(4:6));
end

