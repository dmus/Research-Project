classdef Controller < handle
    %CONTROLLER Controller for a car in the SCR championships.
    %   Controller based on iterative LQR algorithm.
    
    properties
        t               % Current timestep
        sensors         % Current raw sensor readings
        previousSensors % Sensor readings at time t-1
        state           % Current full state
        previousState   % Full state at time t-1
        K               % Optimal time-varying policy
        P               % Time-varying cost-to-go function
        previousAction  % Action executed at time t-1
        model           % Acceleration-model for prediction of next state
    end
    
    methods
        function obj = Controller
            obj.t = 0;
            obj.previousAction = [0 0 0 1]';
            obj.previousState = zeros(18,1);
            obj.state = zeros(18,1);
            
            % Load policy and cost-to-go matrices
            S = load('policy.mat', 'K', 'P');
            obj.K = S.K;
            obj.P = S.P;
            
            % Load Acceleration-model
            addpath('../Acceleration-Model', '../Yaw-Rate-Estimation');
            S = load('AccelerationLaggedModel.mat', 'model');
            obj.model = S.model;
        end
        
        function action = control(this, message)
            % Read all sensor values
            this.previousSensors = this.sensors;
            this.parse(message);
            this.previousState = this.state;
            
            % If start sign is not given yet
            if this.sensors(2) < 0
                this.state = [this.sensors([47 48]); 0; this.sensors([4 69 1]); 1; this.previousState(1:7); this.previousAction];
                action = [0 0 0]';
                return;
            end
            
            % Increase timestep
            this.t = this.t + 1;
            
            % Compute current state
            this.computeState();
            
            % Policy for current timestep is already known, compute change
            % in control inputs
            change = this.K{this.t} * this.state;
            action = this.previousAction + change;
            
            % For next iteration
            this.previousAction = action;
        end
        
        function reset(this)
            disp('resetting...');
        end
        
        function shutdown(this)
            disp('shutdown...');
        end
        
        function parse(this, message)
            this.sensors = sscanf(message, '(angle %f)(curLapTime %f)(damage %f)(distFromStart %f)(distRaced %f)(fuel %f)(gear %f)(lastLapTime %f)(opponents %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f)(racePos %f)(rpm %f)(speedX %f)(speedY %f)(speedZ %f)(track %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f)(trackPos %f)(wheelSpinVel %f %f %f %f)(z %f)(focus %f %f %f %f %f)');
        end
        
        function computeState(this)
            s = zeros(size(this.previousState));
            
            % Forward and lateral speed
            s(1:2) = this.sensors(47:48);
            
            % Angular velocity
            s(3) = this.estimateYawRate();
            
            % State features
            s(4:6) = this.sensors([4 69 1]);
            
            % And remaining elements
            s(7) = 1;
            s(8:14) = this.previousState(1:7);
            
            % Also previous action in state
            s(15:18) = this.previousAction;
            
            this.state = s;
        end
        
        % Estimate yaw rate at time t-1 and use that value to predict
        % current yaw rate.
        function yawRate = estimateYawRate(this)
            % TODO fix dt computation for more than 1 full lap
            dt = this.sensors(2) - this.previousSensors(2);
            
            % Compensation parameter 
            alpha = 0.815; 
            
            % Current guess
            yawRate = 0;
            delta = 0.001;
            epsilon = 0.00001;
            
            % Compute marks at time t-1
            indices = 1:19;
            angles = transp(((indices - 10) / 9) * (0.5*pi) * -1);
            ranges = this.previousSensors(indices + 49);
            marks = [ranges .* cos(angles) ranges .* sin(angles)];
            [marksLeft, marksRight] = groupRangeFinders(marks, 10);

            % Compute move in body coordinate frame
            move = this.previousState(1:2)' .* dt;

            % Compute marks at time t
            newRanges = this.sensors(indices + 49);
            newMarks = [newRanges .* cos(angles) newRanges .* sin(angles)];
            [newMarksLeft, newMarksRight] = groupRangeFinders(newMarks, 10);
            
            % Error with initial guess
            bestError = computeProjectionError(marksLeft, marksRight, move, newMarksLeft, newMarksRight, yawRate);

            % Coordinate descent search
            while delta > epsilon
                yawRate = yawRate + delta;

                % Try with greater value
                error = computeProjectionError(marksLeft, marksRight, move, newMarksLeft, newMarksRight, yawRate);

                if error < bestError
                    bestError = error;
                    delta = delta * 1.1;
                else
                    % If no success true smaller value
                    yawRate = yawRate - (2 * delta);
                    error = computeProjectionError(marksLeft, marksRight, move, newMarksLeft, newMarksRight, yawRate);
                    if error < bestError
                        bestError = error;
                        delta = delta * 1.1;
                    else
                        % If no success, reset yawrate and try with smaller steps next
                        % time
                        yawRate = yawRate + delta;
                        delta = delta * 0.9;
                    end
                end
            end

            yawRate = yawRate * alpha;
            
            % Now we have a better estimate for the yawrate at time t-1
            this.previousState(3) = yawRate;
            
            % Use our acceleration model for a prediction for time t
            % Predict acceleration at time t+tau
            s = this.previousState(1:3);
            
            % a is action at time t-1 converted to other format
            a = this.previousAction;
            a = [a(1) + -1 * a(2), a(3), 1];
    
            acc = zeros(3,1);
            acc(1:2) = this.model.Apos * mapStates(s')' + this.model.Bpos * mapInputs(a, s')';
            acc(3) = this.model.Arot * mapStates(s')' + this.model.Brot * mapInputs(a, s')';
                
            % Compute predicted state at time t with predicted accelerations
            angle = s(3) * dt;
            R = [cos(angle) -sin(angle); sin(angle) cos(angle)];
            s_new(1:2) = R * (s(1:2) + acc(1:2) * dt);
            s_new(3) = s(3) + acc(3) * dt;
            
            % Now we have a prediction for the current yaw rate
            yawRate = s_new(3);
        end
    end
    
end

