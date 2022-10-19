classdef Turtlebot3Waffle < RobotBaseClass
    %% Turtlebot3Waffle
    % This class is based on the Turtlebot3 Waffle. 
    % URL: https://emanual.robotis.com/docs/en/platform/turtlebot3/features/
    % 
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'Turtlebot3Waffle';
    end

    methods (Access = public) 
        %% Define robot Function  
        function self = Turtlebot3Waffle()
            self.CreateModel();
            self.PlotAndColourRobot();
        end

        %% Create the robot model
        function CreateModel(self)       
            L1 = Link('d',0,'a',0,   'alpha',0,'offset',0);
            self.model = SerialLink(L1,'name',self.name);            
        end    
    end
end