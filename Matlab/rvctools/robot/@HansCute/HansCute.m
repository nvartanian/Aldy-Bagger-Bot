classdef HansCute < RobotBaseClass
    %% HansCute
    % This class is based on the HansCute. 
    % URL: http://wiki.ros.org/Robots/Hans-Cute
    % 
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'HansCute';
    end

    methods (Access = public) 
        %% Define robot Function  
        function self = HansCute()
            self.CreateModel();
            self.PlotAndColourRobot();
        end

        %% Create the robot model
        function CreateModel(self)       
            L1 = Link('d',0.15,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset', -pi/2);
            L2 = Link('d',0,'a', 0,'alpha',-pi/2,'qlim', deg2rad([-120 120]), 'offset',0);
            L3 = Link('d',0.125,'a',0,'alpha',pi/2,'qlim', deg2rad([-360 360]), 'offset', 0);
            L4 = Link('d',0,'a',0.065,'alpha',-pi/2,'qlim',deg2rad([-120 120]),'offset', pi/2);
            L5 = Link('d',0,'a',0.065,'alpha',pi/2,'qlim',deg2rad([-120,120]), 'offset',0);
            L6 = Link('d',-0.004,'a',0,'alpha',-pi/2,'qlim',deg2rad([-120,120]), 'offset', -pi/2);
            L7 = Link('d',0.028,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', pi/2);

            self.model = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name',self.name);            
        end    
    end
end