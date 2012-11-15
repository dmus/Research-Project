classdef Controller < handle
    %CONTROLLER Controller for a car in the SCR championships.
    %   Controller based on iterative LQR algorithm.
    
    properties
        t                   % Current timestep
        sensors             % Current raw sensor readings
        previousSensors     % Sensor readings at time t-1
        state               % Current full state
        previousState       % Full state at time t-1
        K                   % Optimal time-varying policy
        P                   % Time-varying cost-to-go function
        previousAction      % Action executed at time t-1
        model               % Acceleration-model for prediction of next state
        H                   % Horizon
        history             % Previous states and actions for this trial
        reference           % Reference trajectory
        Map                 % Map representing the track
        alpha               % Weight parameter for penalizing deviations
        episode             % Number of current trial
        rangeFinderIndices  % Sensor elements corresponding to range finders
        angles              % Range finder configuration
        options             % Options for fminsearch
        A                   % LQR dynamics matrix
        B                   % LQR dynamics matrix
        Q                   % LQR cost matrix
        R                   % LQR cost matrix
        v_prev
    end
    
    methods
        function obj = Controller(H)
            disp('Initializing controller...');
            
            % Horizon
            obj.H = H;
            
            % Load reference trajectory
            S = load('reference.mat', 'reference');
            S.reference.S = S.reference.S(:,1:3);
            obj.reference = S.reference;
            
            stateLength = size(obj.reference.S,2);
            actionLength = size(obj.reference.U,2);
            
            % Load map
            S = load('Map.mat', 'Map');
            obj.Map = S.Map;
            
            % Load Acceleration-model
            addpath('../Acceleration-Model', '../Yaw-Rate-Estimation');
            S = load('AccelerationLaggedModel.mat', 'model');
            obj.model = S.model;
            
            % Initialization
            obj.t = 0;
            obj.episode = 1;
            obj.previousAction = zeros(actionLength,1);
            obj.previousAction(end) = 1;
            obj.previousState = zeros(stateLength,1);
            obj.state = zeros(stateLength,1);
            obj.alpha = 0.9; 
            obj.rangeFinderIndices = (1:19) + 49;
            obj.angles = transp(((obj.rangeFinderIndices - 10) / 9) * (0.5*pi) * -1);
            obj.options = optimset('LargeScale', 'off', 'TolX', 0.0001, 'Display', 'off');
            obj.v_prev = zeros(actionLength,1);
            
            % Compute first time-varying policy
            [obj.A, obj.B, obj.Q, obj.R] = approximateLQR(obj.reference.S(1:H,:), obj.reference.U(1:H-1,:), obj.reference.T(1:H,:), obj.Map, obj.alpha, obj.model);
            [obj.K, obj.P] = createTimeVaryingController(obj.A, obj.B, obj.Q, obj.R, obj.Q{H});
            
            obj.history.S = zeros(obj.H, stateLength);
            obj.history.U = zeros(obj.H, actionLength);
            obj.history.T = zeros(obj.H, 1);
            
            disp('Controller ready...');
        end
        
        function action = control(this, message)
            % Read all sensor values
            this.previousSensors = this.sensors;
            
            if ischar(message)
                this.parse(message);
            else
                this.sensors = message;
            end
            
            this.previousState = this.state;
            
            % If start sign is not given yet
            if this.sensors(2) < 0
                this.state = [this.sensors([47 48]); 0];%; this.sensors([4 69 1]); 1; this.previousState(1:7)];%; this.previousAction];
                action = [0 0 0]';
                return;
            end
            
            % Increase timestep
            this.t = this.t + 1;
            
            % Compute current state
            this.computeState();
            
            this.history.T(this.t,:) = this.sensors(2); % TODO fix for new lap
            this.history.S(this.t,:) = this.state';
            
            if this.t == this.H
                % Output does not matter, because trial has ended
                action = [0 0 0]';
                return;
            end
            
            % Policy for current timestep is already known, compute change
            % in control inputs
            % change = this.K{this.t} * this.state;
            % action = this.previousAction + change;
            z = [this.state - this.reference.S(this.t,:)'; 1; this.v_prev; 1];
            dv = this.K{this.t} * z;
            v = this.v_prev + dv;
            u = v + this.reference.U(this.t,:)';
            
            %u
            
            this.history.U(this.t,:) = u;
            
            % Check if accelerating or braking
            if u(1) > 0
                action = [u(1); 0; u(2)];
            else
                action = [0; -1 * u(1); u(2)];
            end
            
            % For next iteration
            % this.previousAction = u;
            this.v_prev = v;
        end
        
        function reset(this)
            disp('resetting...');
            
            this.alpha = this.alpha - 0.1;
            
            % Compute and solve new optimal control problem           
            [this.A, this.B, this.Q, this.R] = approximateLQR(this.history.S(1:this.H,:), this.history.U(1:this.H-1,:), this.history.T(1:this.H,:), this.Map, this.alpha, this.model);
            [this.K, this.P] = createTimeVaryingController(this.A, this.B, this.Q, this.R, this.Q{this.H});
            
            % Compute total costs
            cost = 0;
            for t = 1:this.H - 1
                cost = cost + g(this.history.S(t,:)') + h(this.history.U(t,:)');
            end
            cost = cost + g(this.history.S(this.H,:)');
            
            fprintf('Final cost for episode %i: %f\n', this.episode, cost);
            %fprintf('Expected: %f\n', this.history.S(this.H,:) * this.P{this.H} * this.history.S(this.H,:)');
            
            this.reference = this.history;
            
            % Reset history and counter
            this.history.S = zeros(size(this.history.S));
            this.history.U = zeros(size(this.history.U));
            this.history.T = zeros(size(this.history.T));
            this.t = 0;
            this.episode = this.episode + 1;
        end
        
        function shutdown(this)
            disp('shutdown...');
        end
        
        function parse(this, message)
            this.sensors = sscanf(message, '(angle %f)(curLapTime %f)(damage %f)(distFromStart %f)(distRaced %f)(fuel %f)(gear %f)(lastLapTime %f)(opponents %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f)(racePos %f)(rpm %f)(speedX %f)(speedY %f)(speedZ %f)(track %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f)(trackPos %f)(wheelSpinVel %f %f %f %f)(z %f)(focus %f %f %f %f %f)');
        end
        
        function computeState(this)
            s = zeros(size(this.previousState));
            
            % Forward and lateral speed in m/s
            s(1:2) = this.sensors(47:48) * 1000 / 3600;
            
            % Angular velocity in rad/s
            s(3) = this.estimateYawRate();

%             % State features
%             s(4:6) = this.sensors([4 69 1]);
%             
%             % And remaining elements
%             s(7) = 1;
%             s(8:14) = this.previousState(1:7);
%             
%             % Also previous action in state
%             %s(15:17) = this.previousAction;
            
            this.state = s;
        end
        
        % Estimate yaw rate at time t-1 and use that value to predict
        % current yaw rate.
        function yawRate = estimateYawRate(this)
            % TODO fix dt computation for more than 1 full lap
            dt = this.sensors(2) - this.previousSensors(2);
            
            % Compensation parameter 
            alpha = 0.805; 
            
            % Current guess
            yawRate = 0;%this.previousState(3);
            
            % Compute marks at time t-1 for the reference scan
            ranges = this.previousSensors(this.rangeFinderIndices);
            marks = [ranges .* cos(this.angles) ranges .* sin(this.angles)];
            
            try
                [marksLeft, marksRight] = groupRangeFinders(marks, 10);

                % Compute move in body coordinate frame
                move = (this.previousState(1:2)' + this.state(1:2)') .* 0.5 .* dt;

                % Compute marks at time t for the current scan
                newRanges = this.sensors(this.rangeFinderIndices);
                newMarks = [newRanges .* cos(this.angles) newRanges .* sin(this.angles)];
                [newMarksLeft, newMarksRight] = groupRangeFinders(newMarks, 10);

                yawRate = fminsearch(@(x) computeProjectionError(marksLeft, marksRight, move, newMarksLeft, newMarksRight, x), yawRate, this.options);

                yawRate = yawRate * alpha;
                
                % Now we have a better estimate for the yawrate at time t-1
                yawRate = yawRate / dt;
            catch e
                disp('Error in yaw rate estimation');
            end
        end
    end
    
end

