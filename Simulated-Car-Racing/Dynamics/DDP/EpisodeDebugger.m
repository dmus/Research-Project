classdef EpisodeDebugger < handle
    %EPISODEPLOTTER For completely debugging an episode.
    %   Plotting and debugging utilities
    
    properties
        episode             % State-input trajectory to debug
        reference           % Reference state-input trajectory
        H                   % Total number of timesteps per episode
        driver              % Controller
        log
        Laps
        start
    end
    
    methods
        function obj = EpisodeDebugger(logFile, reference, H)
            obj.log = load(logFile);
            [obj.Laps, obj.start] = extractLaps(logFile);
            
            % Linear speeds in m/s
            episode.S(:,[1 2]) = obj.Laps{1}.S(1:H, [47 48]) * 1000 / 3600;
            episode.S(:,3) = findYawRates(obj.Laps{1}.S(1:H,:));

%             episode.S(:,4:6) = obj.Laps{1}.S(1:H, [4 69 1]);
%             episode.S(:,7) = 1;
%             
%             % Include previous state
%             episode.S(2:end,8:14) = episode.S(1:end-1,1:7);
%             episode.S(1,8:9) = obj.log.States(obj.start-1,47:48) * 1000 / 3600;
%             episode.S(1,10:14) = [0 obj.log.States(obj.start-1, [4 69 1]) 1];
            
            episode.U = computeActionRepresentation(obj.Laps{1}.A);
            episode.T = computeDiscretizedTimes(obj.Laps{1}.S);
            
            episode.U = episode.U(1:H-1,:);
            episode.T = episode.T(1:H,:);
            
            reference.S = reference.S(1:H,1:3);
            reference.U = reference.U(1:H-1,1:2);
            reference.T = reference.T(1:H,:);
            
            obj.episode = episode;
            obj.reference = reference;
            obj.H = H;
            obj.driver = Controller(H);
        end
        
        function plotInputs(this)
            H = this.H;
            figure;
            plot(1:H-1,this.episode.U(:,1),'-r',1:H-1,this.episode.U(:,2),'-b');
            hold on;
            plot(1:H-1,this.reference.U(:,1),'--r',1:H-1,this.reference.U(:,2),'--b');
            legend('Acceleration','Steering','ref','ref');
            hold off;
        end
        
        function plotStates(this)
            figure;
            subplot(2,2,1);
            plot(1:this.H,this.episode.S(:,5),'-r', 1:this.H,this.reference.S(:,5),'--r');
            hold on;
            plot(1:this.H,this.episode.S(:,6),'-b', 1:this.H,this.reference.S(:,6),'--b');
            legend('trackPos','ref','angle','ref');
            hold off;
            
            subplot(2,2,2);
            plot(1:this.H,this.episode.S(:,4),'-r', 1:this.H,this.reference.S(:,4),'--r');
            legend('distFromStart','ref');
            
            % Plot velocities
            subplot(2,2,3);
            hold on;
            plot(1:this.H,this.episode.S(:,2),'-b', 1:this.H,this.reference.S(:,2),'--b');
            plot(1:this.H,this.episode.S(:,3),'-g', 1:this.H,this.reference.S(:,3),'--g');
            legend('speedY','ref','angularSpeed','ref');
            hold off;
            
            subplot(2,2,4);
            plot(1:this.H,this.episode.S(:,1),'-r', 1:this.H,this.reference.S(:,1),'--r');
            legend('speedX','ref');
        end
        
        function plotCosts(this)
            figure;
            % Compute cost for episode
            cost = zeros(1,this.H);
            refCost = zeros(1,this.H);
            
            prevCost = 0;
            prevRefCost = 0;
            for t = 1:this.H-1
                cost(t) = prevCost + g(this.episode.S(t,:)) + h(this.episode.U(t,:)');
                refCost(t) = prevRefCost + g(this.reference.S(t,:)) + h(this.reference.U(t,:)');
                
                prevCost = cost(t);
                prevRefCost = refCost(t);
            end
            
            cost(this.H) = cost(this.H-1) + g(this.episode.S(this.H,:));
            refCost(this.H) = refCost(this.H-1) + g(this.reference.S(this.H,:));
            
            plot(1:this.H,cost,'-r',1:this.H,refCost,'--r');
            legend('Cost','ref');
        end
        
        function testDynamics(this)
            for t = 1:this.H-1
                dt = this.episode.T(t+1) - this.episode.T(t);
                
                z_t = this.episode.S(t,:)' - this.reference.S(t,:)';
                v_t = this.episode.U(t,:)' - this.reference.U(t,:)';
                z_tplus1 = this.driver.A{t} * z_t + this.driver.B{t} * v_t;
                
                s_tplus1 = z_tplus1 + this.reference.S(t,:)';
                s_next = f(this.episode.S(t,:)', this.episode.U(t,:)', dt, this.driver.Map);
                target = this.episode.S(t+1,:)';
            end
        end
        
        function testCosts(this)
            for t = 1:this.H-1
                % Test Q matrix
                s = this.episode.S(t,:)';
                z = s - this.reference.S(t,:)';
                cost = g(s);
                penalty = norm(z)^2;
                
                alpha = 0.99;
                cost = (1-alpha) * cost + alpha * penalty;
                
                approximation = z' * this.driver.Q{t} * z;
                diff = cost - approximation;
                
                % Same for R matrix
                u = this.episode.U(t,:)';
                v = u - this.reference.U(t,:)';
                cost = h(u);
                penalty = norm(v)^2;
                
                alpha = 0.99;
                cost = (1-alpha) * cost + alpha * penalty;
                
                approximation = v' * this.driver.R{t} * v;
                diff = cost - approximation;
            end
        end
        
        function simulate(this)
            for i = 1:this.start-1
                this.driver.control(this.log.States(i,:)');
            end
            
            for t = 1:this.H-1
%                 z_t = this.episode.S(t,:)' - this.reference.S(t,:)';
%                 z_tplus1 = this.episode.S(t,:)' - this.reference.S(t,:)';
%                 
                v_prev = this.driver.v_prev;
                action = this.driver.control(this.log.States(this.start - 1 + t,:)');
                u = [action(1) + -1 * action(2); action(3)];
                
                s = this.driver.state;
                
                z_t = [s - this.reference.S(t,:)'; 1; v_prev; 1];
                dv = this.driver.K{t} * z_t;
                v_t = v_prev + dv;
                
                next = this.driver.A{t} * z_t + this.driver.B{t} * dv;
                result = z_t' * this.driver.Q{t} * z_t + dv' * this.driver.R{t} * dv + next' * this.driver.P{t+1} * next;
                fun = @(x) z_t' * this.driver.Q{t} * z_t + x' * this.driver.R{t} * x + (this.driver.A{t} * z_t + this.driver.B{t} * x)' * this.driver.P{t+1} * (this.driver.A{t} * z_t + this.driver.B{t} * x);
                v = fminsearch(fun,[0 0]');
            end
            
            this.driver.t = 0;
        end
    end
    
end

