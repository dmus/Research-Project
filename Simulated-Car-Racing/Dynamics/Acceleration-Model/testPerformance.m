function [velocityError, angularRateError] = testPerformance(model, H, S, U, times)
%TESTPERFORMANCE Tests performance of an acceleration model on a testrun.
%   TESTPERFORMANCE measures average squared prediction errors between true
%   state and simulated state. The test data in TESTRUN is split into
%   consecutive non-overlapping windows which corresponds to H simulation
%   steps.
    
    % States are divided in non-overlapping windows
    numWindows = 0;
    
    % Errors for each simulation step
    velocityError = zeros(1,H);
    angularRateError = zeros(1,H);
    
    % Start at first state, stop if no window of H timesteps is left
    t = 1;
    while (t + H) <= size(S,1)
        % Given state
        start = S(t,:);
        
        for j = t+1:t+H
            % Predict state at time j given state at time t
            truth = S(j,:)';
            
            % Number of simulation steps needed
            h = j-t;
            state = start';
            
            for tau = 0:h-1
                % Time difference between state at time t+tau and t+tau+1
                dt = times(t+tau+1) - times(t+tau);
                
                % Predict acceleration at time t+tau
                acc = zeros(3,1);
                acc(1:2) = model.Apos * mapStates(state')' + model.Bpos * mapInputs(U(t+tau,:),state')';
                acc(3) = model.Arot * mapStates(state')' + model.Brot * mapInputs(U(t+tau,:),state')';
                
                % Compute state at time t+tau+1 with predicted
                % accelerations
                R = [cos(state(3)) -sin(state(3)); sin(state(3)) cos(state(3))];
                state(1:2) = R * (state(1:2) + acc(1:2) * dt);
                state(3) = state(3) + acc(3) * dt;
            end
            predicted = state;
            
            velocityError(h) = velocityError(h) + sum((truth(1:2) - predicted(1:2)) .^2);
            angularRateError(h) = angularRateError(h) + sum((truth(3) - predicted(3)) .^2);
        end
        
        numWindows = numWindows + 1;
        
        % Go to next window
        t = t + H;
    end
    
    % Compute error means
    velocityError = velocityError ./ numWindows;
    angularRateError = angularRateError ./ numWindows;
end

