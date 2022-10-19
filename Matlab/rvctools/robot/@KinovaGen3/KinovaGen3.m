classdef KinovaGen3 < RobotBaseClass
    %% KinovaGen3
    % This class is based on the KinovaGen3. 
    % URL: https://www.kinovarobotics.com/product/gen3-robots
    % 
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'KinovaGen3';
    end

    methods (Access = public) 
        %% Define robot Function  
        function self = KinovaGen3()
            self.CreateModel();
            self.PlotAndColourRobot();
        end

        %% Create the robot model
        function CreateModel(self)       
            L1 = Link('d',0.2433,'a',0,   'alpha',pi/2,'offset',pi/2,'qlim',[deg2rad(-154.1) ,deg2rad(154.1)]);
            L2 = Link('d',0.03,  'a',0.28,'alpha',0,   'offset',pi/2,'qlim',[deg2rad(-150.1) ,deg2rad(150.1)]);
            L3 = Link('d',-.02,  'a',0,   'alpha',pi/2,'offset',-pi/2,'qlim',[deg2rad(-150.1) ,deg2rad(150.1)]);
            L4 = Link('d',-.245, 'a',0,   'alpha',pi/2,'offset',pi/2,'qlim',[deg2rad(-148.98),deg2rad(148.98)]);
            L5 = Link('d',0.057, 'a',0,   'alpha',pi/2,'offset',0,'qlim',[deg2rad(-144.97),deg2rad(145.0)]);
            L6 = Link('d',0.2622,'a',0,   'alpha',0,   'offset',0,'qlim',[deg2rad(-148.98),deg2rad(148.98)]);
            
            self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',self.name);            
        end    
    end
end
