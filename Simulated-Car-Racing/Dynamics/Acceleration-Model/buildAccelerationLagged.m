function model = buildAccelerationLagged(trainrun, H)
%BUILDACCELERATIONLAGGED Builds a dynamics model.
%   BUILDACCELERATIONLAGGED buidls a acceleration-based dynamics model
%   taking into account H simulation steps. However, the result is still a
%   first-order Markov model.

    % 1. Minimize one-step squared prediction error to obtain initial model
    disp('1. Building initial model');
    
    [initialModel, S, U, times, Accelerations] = buildAccelerationOneStep(trainrun);
    
    i = 1;
    epsilon = 0.1;
    
    model{i} = initialModel;
    
    stop = false;
    while ~stop
        % 2. Simulate in current model
        disp('2. Simulating');
        
        T = size(S,1);
        Predictions = zeros(T, H, 3);

        for t = 1:T-H
            for h = 1:H
                % Predict s at time t+h given s at time t
                s = S(t,:)';
                for tau=0:h-1
                    % Predict acceleration
                    accelerations = zeros(3,1);
                    accelerations(1:2) = model{i}.Apos * s + model{i}.Bpos * U(t+tau,:)';
                    accelerations(3) = model{i}.Arot * s + model{i}.Brot * U(t+tau,:)';

                    yawrate = s(3);
                    R = [cos(yawrate) -sin(yawrate); sin(yawrate) cos(yawrate)];

                    % Compute new state
                    dt = times(t+tau+1) - times(t+tau);
                    s(1:2) = R * (s(1:2) + accelerations(1:2) * dt);
                    s(3) = s(3) + accelerations(3) * dt;
                end

                % Store prediction
                Predictions(t,h,:) = s;
            end
        end
        
        % 3. Solve 3 least-squares problem
        disp('3. Solving least-squares');
        
        % Construct datasets
        Xx = zeros((T - H) * H, 7);
        Xy = zeros((T - H) * H, 7);
        Xomega = zeros((T - H) * H, 7);
        Y = zeros((T - H) * H, 3);

        % Start with filling the first row, k=1
        k = 1;
        for t = 1:T-H
            for h = 1:H
                y = zeros(1,3);
                xx = zeros(1,7);
                xy = zeros(1,7);
                xomega = zeros(1,7);
                
                % Angle made so far in current simulation
                yaw = 0;
                
                for tau = 0:h-1 
                    dt = times(t+tau+1) - times(t+tau);
                    
                    % Rotate back to start of current simulation
                    R = [cos(-yawrate) -sin(-yawrate); sin(-yawrate) cos(-yawrate)];
                    
                    % s is current state
                    if tau > 0
                        s = Predictions(t,tau,:);
                        s = s(:)';
                    else
                        s = S(t,:);
                    end   

                    xx = xx + ([s U(t+tau,:)] * R(1,1) + [s U(t+tau,:)] * R(1,2)) * dt;
                    xy = xy + ([s U(t+tau,:)] * R(2,1) + [s U(t+tau,:)] * R(2,2)) * dt;
                    xomega = xomega + ([s U(t+tau,:)]) * dt;
                    
                    y(1:2) = y(1:2) + (R * Accelerations(t+tau,1:2)')' * dt;
                    y(3) = y(3) + Accelerations(t+tau,3);
                    
                    % Update angle made so far
                    yaw = yaw + s(3);
                end
                
                % Put in datasets
                Xx(k,:) = xx;
                Xy(k,:) = xy;
                Xomega(k,:) = xomega;
                Y(k,:) = y;
                k = k + 1;
            end
        end

        % Now we have the datasets for 3 linear least squares problems
        Apos = zeros(size(model{i}.Apos));
        Bpos = zeros(size(model{i}.Bpos));
        Arot = zeros(size(model{i}.Arot));
        Brot = zeros(size(model{i}.Brot));
        
        % Exclude steering command
        Xx(:,6) = 0;
        theta = linearRegression(Xx,Y(:,1));
        
        Apos(1,:) = theta(1:3);
        Bpos(1,:) = theta(4:7);
        
        theta = linearRegression(Xy,Y(:,2));
        Apos(2,:) = theta(1:3);
        Bpos(2,:) = theta(4:7);
        
        theta = linearRegression(Xomega,Y(:,3));
        Arot(1,:) = theta(1:3);
        Brot(1,:) = theta(4:7);
        
        % 4. Update A,B
        disp('4. Updating model');
        
        % TODO linesearch to choose alpha
        alpha = 0.1;

        model{i+1}.Apos = (1-alpha) * model{i}.Apos + alpha * Apos;
        model{i+1}.Bpos = (1-alpha) * model{i}.Bpos + alpha * Bpos;
        model{i+1}.Arot = (1-alpha) * model{i}.Arot + alpha * Arot;
        model{i+1}.Brot = (1-alpha) * model{i}.Brot + alpha * Brot;
        
        % 5. Check if converged
        disp('5. Check stop condition');
        
        change = 0;
        change = change + sqrt(sum((model{i+1}.Apos(:) - model{i}.Apos(:)) .^2));
        change = change + sqrt(sum((model{i+1}.Bpos(:) - model{i}.Bpos(:)) .^2));
        change = change + sqrt(sum((model{i+1}.Arot(:) - model{i}.Arot(:)) .^2));
        change = change + sqrt(sum((model{i+1}.Brot(:) - model{i}.Brot(:)) .^2));
        
        fprintf('Change: %f\n', change)
        if change < epsilon
            stop = true;
        end
        
        i = i + 1;
    end
end

