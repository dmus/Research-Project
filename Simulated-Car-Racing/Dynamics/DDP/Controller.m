classdef Controller < handle
    %CONTROLLER Controller for a car in the SCR championships.
    %   Controller based on iterative LQR algorithm.
    
    properties
        t                   % Current timestep
        sensors             % Current raw sensor readings
        sensors_prev        % Sensor readings at time t-1
        state               % Current full state
        state_prev       % Full state at time t-1
        K                   % Optimal time-varying policy
        P                   % Time-varying cost-to-go function
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
        state_dim           % Number of elements in state
        action_dim          % Number of elements in action
        bias
        use_bias            % Setting for using inaccurate model algoritm or not
    end
    
    methods
        function obj = Controller(H)
            disp('Initializing controller...');
            
            % Horizon
            obj.H = H;
            
            % Load reference trajectory
            S = load('reference2.mat', 'reference');
            S.reference.S = S.reference.S(1:H,1:3);
            S.reference.U = S.reference.U(1:H-1,1:2);
            S.reference.T = S.reference.T(1:H);
            obj.reference = S.reference;
            
            obj.state_dim = size(obj.reference.S,2);
            obj.action_dim = size(obj.reference.U,2);
            
            % Load map
            S = load('Map.mat', 'Map');
            obj.Map = S.Map;
            
            % Load Acceleration-model
            addpath('../Acceleration-Model', '../Yaw-Rate-Estimation');
            S = load('AccelerationLaggedModel.mat', 'model');
            obj.model = S.model;
            
            % Initialization
            obj.episode = 0;
            %obj.previousState = zeros(stateLength,1);
            
            obj.alpha = 0; 
            obj.rangeFinderIndices = (1:19) + 49;
            obj.angles = transp(((obj.rangeFinderIndices - 10) / 9) * (0.5*pi) * -1);
            obj.options = optimset('LargeScale', 'off', 'TolX', 0.0001, 'Display', 'off');
            obj.use_bias = true;
            
            obj.onStart();
            
            disp('Controller ready...');
        end
        
        % Initialize state of driver for a new episode
        function onStart(this)
            this.t = 0;
            this.episode = this.episode + 1;
            this.v_prev = zeros(this.action_dim,1);
            this.state = zeros(this.state_dim,1);
            
            this.history.S = zeros(this.H, this.state_dim);
            this.history.U = zeros(this.H, this.action_dim);
            this.history.T = zeros(this.H, 1);
            
            % Compute first time-varying policy
            if this.use_bias
                % Compute bias for time-dependent dynamics function
                this.bias = zeros(this.H-1, this.state_dim);
                for t = 1:this.H - 1
                    dt = this.reference.T(t+1) - this.reference.T(t);
                    prediction = f(this.reference.S(t,:)', this.reference.U(t,:)', dt, this.Map, this.model);
                    this.bias(t,:) = this.reference.S(t+1,:) - prediction';
                end
                
            else
                this.bias = zeros(this.H-1, this.state_dim);
            end
                
            [this.A, this.B, this.Q, this.R] = approximateLQR(this.reference.S, this.reference.U, this.reference.T, this.Map, this.alpha, this.model, this.bias);
            [this.K, this.P] = createTimeVaryingController(this.A, this.B, this.Q, this.R, this.Q{this.H});
        end
        
        % Compute action for a call with a string message with all sensor
        % readings
        function action = controlFromMessage(this, message)
            sensors = this.parse(message);
            u = this.control(sensors);
            
            % Check if accelerating or braking
            if u(1) > 0
                action = [u(1); 0; u(2)];
            else
                action = [0; -1 * u(1); u(2)];
            end
        end
        
        % Sensors is a long vector containing sensor measurements
        function u = controlFromSensors(this, sensors)
            % Save previous sensor readings
            this.sensors_prev = this.sensors;
            this.sensors = sensors;
            
            % If start sign is not given yet
            if this.sensors(2) < 0
                this.state = [this.sensors([47 48]); 0];%; this.sensors([4 69 1]); 1; this.previousState(1:7)];%; this.previousAction];
                u = zeros(this.action_dim,1);
                return;
            end
            
            % Compute current state and compute control action
            s = this.computeState();
            u = this.control(s, this.sensors(2)); % TODO fix for new lap
        end
        
        function u = control(this, state, curtime)
            % Increase timestep
            this.t = this.t + 1;
            
            % Save previous state
            this.state_prev = this.state;
            this.state = state;
            
            this.history.T(this.t,:) = curtime;
            this.history.S(this.t,:) = this.state';
            
            if this.t >= this.H
                % Output does not matter, because trial has ended
                u = zeros(this.action_dim,1);
                return;
            end
            
            % Compute change in control inputs
            z = [this.state - this.reference.S(this.t,:)'; 1; this.v_prev; 1];
            dv = this.K{this.t} * z;
            v = this.v_prev + dv;
            u = v + this.reference.U(this.t,:)';
            
            this.history.U(this.t,:) = u;
            
            % For next timestep
            this.v_prev = v;
        end
        
        function reset(this)
            disp('resetting...');
            
            % Compute total costs
            cost = 0;
            for t = 1:this.H - 1
                cost = cost + g(this.history.S(t,:)') + h(this.history.U(t,:)');
            end
            cost = cost + g(this.history.S(this.H,:)');
            
            fprintf('Final cost for episode %i: %f\n', this.episode, cost);
            
            this.alpha = this.alpha - 0.1;
            
            % Compute and solve new optimal control problem
            
            % this.reference = this.history;
            
            this.onStart();
        end
        
        function shutdown(this)
            disp('Shutdown...');
        end
        
        function sensors = parse(this, message)
            sensors = sscanf(message, '(angle %f)(curLapTime %f)(damage %f)(distFromStart %f)(distRaced %f)(fuel %f)(gear %f)(lastLapTime %f)(opponents %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f)(racePos %f)(rpm %f)(speedX %f)(speedY %f)(speedZ %f)(track %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f)(trackPos %f)(wheelSpinVel %f %f %f %f)(z %f)(focus %f %f %f %f %f)');
        end
        
        function s = computeState(this)
            s = zeros(size(this.state));
            
            % Forward and lateral speed in m/s
            s(1:2) = this.sensors(47:48) * 1000 / 3600;
            %s(1) = this.sensors(47) * 1000 / 3600;
            
            
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
        end
        
        % Estimate yaw rate at time t.
        function yawRate = estimateYawRate(this)
            % TODO fix dt computation for more than 1 full lap
            dt = this.sensors(2) - this.sensors_prev(2);
            
            % Compensation parameter 
            alpha = 0.805; 
            
            % Current guess
            yawRate = 0;%this.previousState(3);
            
            % Compute marks at time t-1 for the reference scan
            ranges = this.sensors_prev(this.rangeFinderIndices);
            marks = [ranges .* cos(this.angles) ranges .* sin(this.angles)];
            
            try
                [marksLeft, marksRight] = groupRangeFinders(marks, 10);

                % Compute move in body coordinate frame
                move = (this.state_prev(1:2)' + this.state(1:2)') .* 0.5 .* dt;

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
