classdef Controller < handle
    %CONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        t = 0;
    end
    
    methods
        function obj = Controller
            % initialization
            disp('init...');
        end
        
        function action = control(this, message)
            disp(message);
            this.t = this.t + 1;
            action = [1 0 1]';
        end
        
        function reset(this)
            disp('resetting...');
        end
        
        function shutdown(this)
            disp('shutdown...');
        end
        
        function parse(this, message)
        end
    end
    
end

