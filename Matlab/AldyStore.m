classdef AldyStore
    properties
        robot;
        
        %status
        eStop = false;
        
    end
    methods
        %constructor
        function obj = AldyStore(robot)
            if nargin == 1
                obj.robot = robot;
            end
            
            %setup environment
            %add floor
            
            %add checkout
            
            %safety features
            %e-stop
            
            %light curtain
            
        end
        
    end
end