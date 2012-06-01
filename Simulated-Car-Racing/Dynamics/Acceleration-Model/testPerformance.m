function error = testPerformance(model, H, S, U, times)
%TESTPERFORMANCE Tests performance of an acceleration model on a testrun.
%   TESTPERFORMANCE measures average squared prediction errors between true
%   state and simulated state. The test data in TESTRUN is split into
%   consecutive non-overlapping windows which corresponds to H simulation
%   steps.
    
    % States are divided in non-overlapping windows
    numWindows = 0;
    
    % Errors for each simulation step
    error.x = zeros(1,H);
    error.y = zeros(1,H);
    error.omega = zeros(1,H);
    
    error.n = zeros(1,H);
    error.e = zeros(1,H);
    error.o = zeros(1,H);
    
    % Start at first state, stop if no window of H timesteps is left
    t = 1;
    while (t + H) <= size(S,1)
        % Given state
        start = S(t,:);
        start(4:6) = 0;
        
        for j = t+1:t+H
            % Predict state at time j given state at time t
            %truth = S(j,:)';
            
            % Number of simulation steps needed
            h = j-t;
            
            S(j-h,4:6) = 0;
            
            
            state = start';
                    
            
            for tau = 0:h-1
                % Time difference between state at time t+tau and t+tau+1
                dt = times(t+tau+1) - times(t+tau);
                
                % New true (estimated) position + orientation
                yaw = S(j-h+tau,6);
                R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
                S(j-h+tau+1,4:5) = S(j-h+tau,4:5) + (R * (S(j-h+tau,1:2) * dt)')';
                S(j-h+tau+1,6) = S(j-h+tau,6) + S(j-h+tau,3)*dt;
                
                % Predicted position + orientation
                yaw = state(6);
                R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
                state(4:5) = state(4:5) + (R * (state(1:2) * dt));
                state(6) = state(6) + state(3)*dt;
                
                % Predict acceleration at time t+tau
                acc = zeros(3,1);
                acc(1:2) = model.Apos * mapStates(state')' + model.Bpos * mapInputs(U(t+tau,:),state')';
                acc(3) = model.Arot * mapStates(state')' + model.Brot * mapInputs(U(t+tau,:),state')';
                
                % Compute state at time t+tau+1 with predicted
                % accelerations
                a = state(3) * dt;
                R = [cos(a) -sin(a); sin(a) cos(a)];
                state(1:2) = R * (state(1:2) + acc(1:2) * dt);
                state(3) = state(3) + acc(3) * dt;
            end
            predicted = state;
            
            truth = S(j,:)';
            
            error.x(h) = error.x(h) + sum((truth(1) - predicted(1)) .^2);
            error.y(h) = error.y(h) + sum((truth(2) - predicted(2)) .^2);
            error.omega(h) = error.omega(h) + sum((truth(3) - predicted(3)) .^2);
            
            error.n(h) = error.n(h) + sum((truth(4) - predicted(4)) .^2);
            error.e(h) = error.e(h) + sum((truth(5) - predicted(5)) .^2);
            error.o(h) = error.o(h) + sum((truth(5) - predicted(5)) .^2);
        end
        
        numWindows = numWindows + 1;
        
        % Go to next window
        t = t + H;
    end
    
    % Compute error means
    error.x = error.x ./ numWindows;
    error.y = error.y ./ numWindows;
    error.omega = error.omega ./ numWindows;
    
    error.n = error.n ./ numWindows;
    error.e = error.e ./ numWindows;
    error.o = error.o ./ numWindows;
end

