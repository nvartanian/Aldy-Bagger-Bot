classdef AldyStore
    properties
        AldyBaggerBot;
        ConveyorBelt; 
        BaggingArea; %need class for baggingArea and Bag
        
        %status
        eStop = false;
        
    end
    methods
        %constructor
        function obj = AldyStore(AldyBaggerBot)
            if nargin == 1
                obj.AldyBaggerBot = AldyBaggerBot;
            end
            obj.ConveyorBelt = ConveyorBelt(); %create ConveyorBelt obj
            obj.BaggingArea = BaggingArea(); %create BaggingArea obj
            
            %setup environment
            %add floor
            
            %add checkout
            
            %safety features
            %e-stop
            
            %light curtain
            
        end
        
        function self = stepStore(self)
            self.ConveyorBelt.stepBeltX();
            self.AldyBaggerBot.stepRobot();
        end
        
    end
end