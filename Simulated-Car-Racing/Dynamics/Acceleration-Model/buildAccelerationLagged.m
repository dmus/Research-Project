function model = buildAccelerationLagged(S, U, times, H)
%BUILDACCELERATIONLAGGED Builds a dynamics model.
%   BUILDACCELERATIONLAGGED buidls a acceleration-based dynamics model
%   taking into account H simulation steps. However, the result is still a
%   first-order Markov model.

    % 1. Minimize one-step squared prediction error to obtain initial model
    disp('1. Building initial model');
    
    [initialModel, Accelerations] = buildAccelerationOneStep(S, U, times);
    
    i = 1;
    epsilon = 0.1;
    T = size(S,1); % Number of timesteps
    
    model{i} = initialModel;
    
    stop = false;
    while ~stop
        % 2. Simulate in current model
        disp('2. Simulating');
        [Predictions, error] = simulate(model{i}, H, S, U, times);
        cost{i} = error;
        
        fprintf('Simulation error: %f\n', error);
        
        % 3. Solve 3 least-squares problems
        disp('3. Solving least-squares');
        
        numStateFeatures = 6;
        numInputFeatures = 8;

        % Construct datasets
        X_x = zeros((T - H) * H, numStateFeatures+numInputFeatures);
        X_y = zeros((T - H) * H, numStateFeatures+numInputFeatures);
        X_omega = zeros((T - H) * H, numStateFeatures+numInputFeatures);
        Y = zeros((T - H) * H, 3);

        % Start with filling the first row, k=1
        k = 1;
        for t = 1:T-H
            for h = 1:H
                y = zeros(1,3);
                x_x = zeros(1,numStateFeatures+numInputFeatures);
                x_y = zeros(1,numStateFeatures+numInputFeatures);
                x_omega = zeros(1,numStateFeatures+numInputFeatures);
                
                % Angle made so far in current simulation
                yaw = 0;
                
                for tau = 0:h-1 
                    dt = times(t+tau+1) - times(t+tau);
                    
                    % Rotate back to start of current simulation
                    R = [cos(-yaw) -sin(-yaw); sin(-yaw) cos(-yaw)];
                    
                    % s is current state
                    if tau > 0
                        s = Predictions(t,tau,:);
                        s = s(:)';
                    else
                        s = S(t,:);
                    end   

                    % Rotate and scale state and input features
                    sf = mapStates(s);
                    uf = mapInputs(U(t+tau,:),s);

                    x_x = x_x + ([sf uf] * R(1,1) + [sf uf] * R(1,2)) * dt;
                    x_y = x_y + ([sf uf] * R(2,1) + [sf uf] * R(2,2)) * dt;
                    x_omega = x_omega + ([sf uf]) * dt;
                    
                    y(1:2) = y(1:2) + (R * Accelerations(t+tau,1:2)')' * dt;
                    y(3) = y(3) + Accelerations(t+tau,3) * dt;
                    
                    % Update angle made so far
                    yaw = yaw + s(3) * dt;
                end
                
                % Put in datasets
                X_x(k,:) = x_x;
                X_y(k,:) = x_y;
                X_omega(k,:) = x_omega;
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
        X_x(:,[2 3 4 9:end]) = 0;
        theta = linearRegression(X_x,Y(:,1));
        
        Apos(1,:) = theta(1:numStateFeatures);
        Bpos(1,:) = theta(numStateFeatures+1:end);
        
        X_y(:,[1 4:6 7 8 12 13]) = 0;
        theta = linearRegression(X_y,Y(:,2));
        Apos(2,:) = theta(1:numStateFeatures);
        Bpos(2,:) = theta(numStateFeatures+1:end);
        
        X_omega(:,[1 2 5 6 7 8 10 11]) = 0;
        theta = linearRegression(X_omega,Y(:,3));
        Arot(1,:) = theta(1:numStateFeatures);
        Brot(1,:) = theta(numStateFeatures+1:end);
        
        % 4. Update A,B
        disp('4. Updating model');
        
        % Linesearch to choose stepsize alpha
        alpha = 1;
        
        improved = false;
        while ~improved
            temp.Apos = (1-alpha) * model{i}.Apos + alpha * Apos;
            temp.Bpos = (1-alpha) * model{i}.Bpos + alpha * Bpos;
            temp.Arot = (1-alpha) * model{i}.Arot + alpha * Arot;
            temp.Brot = (1-alpha) * model{i}.Brot + alpha * Brot;
            
            [~, error] = simulate(temp, H, S, U, times);
            fprintf('Simulation error: %f\n', error);
            
            if error < cost{i}
                improved = true;
            else
                alpha = 0.5 * alpha;
                if alpha <= 1/2^15
                    break;
                end
            end
        end

        fprintf('Stepsize alpha: %f\n', alpha);
        
        if improved
            model{i+1}.Apos = temp.Apos;
            model{i+1}.Bpos = temp.Bpos;
            model{i+1}.Arot = temp.Arot;
            model{i+1}.Brot = temp.Brot;
        else
            model{i+1} = model{i};
        end
            
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

