function [Predictions, error] = simulate(model, H, S, U, times)
%SIMULATE Simulate in MODEL to compute states up to H steps forward.
%   SIMULATE uses MODEL which contains four matrices to compute state
%   predictions and an error measure.
    error = 0;
        
    T = size(S,1);
    Predictions = zeros(T, H, 3);

    for t = 1:T-H
        for h = 1:H
            % Predict s at time t+h given s at time t
            s = S(t,:)';
            for tau=0:h-1
                % Predict acceleration

                sf = mapStates(s')';
                uf = mapInputs(U(t+tau,:)',s')';

                accelerations = zeros(3,1);
                accelerations(1:2) = model.Apos * sf + model.Bpos * uf;
                accelerations(3) = model.Arot * sf + model.Brot * uf;

                yawrate = s(3);
                R = [cos(yawrate) -sin(yawrate); sin(yawrate) cos(yawrate)];

                % Compute new state
                dt = times(t+tau+1) - times(t+tau);
                s(1:2) = R * (s(1:2) + accelerations(1:2) * dt);
                s(3) = s(3) + accelerations(3) * dt;
            end

            % Store prediction
            Predictions(t,h,:) = s;
            error = error + sum((s - S(t+h,:)') .^2);
        end
    end

end

