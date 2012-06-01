function [Predictions, error] = simulate(model, H, S, U, times)
%SIMULATE Simulate in MODEL to compute states up to H steps forward.
%   SIMULATE uses MODEL which contains four matrices to compute state
%   predictions and an error measure.
    error = 0;
        
    T = size(S,1);
    Predictions = zeros(T, H, 6);

    for t = 1:T-H
        for h = 1:H
            if h == 1
                S(t+h-1,4:6) = 0;
                s = S(t,:)';
            else
                s = Predictions(t,h-1,:);
                s = s(:);
            end
            
            dt = times(t+h) - times(t+h-1);
            
            % Set estimated true position
            yaw = S(t+h-1,6);
            R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
            S(t+h,4:5) = S(t+h-1,4:5) + (R * (S(t+h-1,1:2) * dt)')';
            S(t+h,6) = S(t+h-1,6) + S(t+h-1,3) * dt; 
            
            sf = mapStates(s')';
            uf = mapInputs(U(t+h-1,:),s')';
            
            
            accelerations = zeros(3,1);
            accelerations(1:2) = model.Apos * sf + model.Bpos * uf;
            accelerations(3) = model.Arot * sf + model.Brot * uf;

            
            
            % New position + orientation 
            R = [cos(-s(6)) -sin(-s(6)); sin(-s(6)) cos(-s(6))];
            s(4:5) = s(4:5) + (R * (s(1:2) * dt));
            s(6) = s(6) + s(3) * dt;
            
            %
            
            yawrate = s(3);
            
            a = yawrate * dt;
            R = [cos(a) -sin(a); sin(a) cos(a)];

            % Compute new velocities
            s(1:2) = R * (s(1:2) + accelerations(1:2) * dt);
            s(3) = s(3) + accelerations(3) * dt;

            
            
            % Store prediction
            Predictions(t,h,:) = s;
            
            e = s - S(t+h,:)';
            e(3) = e(3) .* 10;
            error = error + sum(e .^2);
        end
    end

end

