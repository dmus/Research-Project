function [Predictions, error] = simulate(model, H, S, U, times)
%SIMULATE Simulate in MODEL to compute states up to H steps forward.
%   SIMULATE uses MODEL which contains four matrices to compute state
%   predictions and an error measure.
    error = 0;
        
    T = size(S,1);
    Predictions = zeros(T, H, 3);

    for t = 1:T-H
        for h = 1:H
            if h == 1
                s = S(t,:)';
            else
                s = Predictions(t,h-1,:);
                s = s(:);
            end
            
            sf = mapStates(s')';
            uf = mapInputs(U(t+h-1,:),s')';
            
            accelerations = zeros(3,1);
            accelerations(1:2) = model.Apos * sf + model.Bpos * uf;
            accelerations(3) = model.Arot * sf + model.Brot * uf;

            yawrate = s(3);
            dt = times(t+h) - times(t+h-1);
            a = yawrate * dt;
            R = [cos(a) -sin(a); sin(a) cos(a)];

            % Compute new state
            s(1:2) = R * (s(1:2) + accelerations(1:2) * dt);
            s(3) = s(3) + accelerations(3) * dt;

            % Store prediction
            Predictions(t,h,:) = s;
            error = error + sum((s - S(t+h,:)') .^2);
        end
    end

end

