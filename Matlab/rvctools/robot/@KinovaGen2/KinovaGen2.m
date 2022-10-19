classdef KinovaGen2 < RobotBaseClass
    %% KinovaGen2
    % This class is based on the KinovaGen2. 
    % URL: https://www.kinovarobotics.com/product/gen2-robots
    % 
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access =public)   
        plyFileNameStem = 'KinovaGen2';
    end

    methods (Access = public) 
        %% Define robot Function  
        function self = KinovaGen2()
            self.CreateModel();
            self.PlotAndColourRobot();
        end

        %% Create the robot model
        function CreateModel(self)       
            L1 = Link('d',0.2755,   'a',0,   'alpha',pi/2,'offset',0,    'qlim',[-720  720]*pi/180);
            L2 = Link('d',0,        'a',0.410,'alpha',-pi,'offset',0,'qlim',[  47  313]*pi/180);
            L3 = Link('d',0.013,    'a',0,   'alpha',pi/2,'offset',0,'qlim',[  19  341]*pi/180);
            L4 = Link('d',0.3111 ,  'a',0,   'alpha',pi/2,'offset',0,    'qlim',[-720  720]*pi/180);
            L5 = Link('d',0,        'a',0,   'alpha',pi/2,'offset',pi,    'qlim',[  65  295]*pi/180);
            L6 = Link('d',0.2638,   'a',0,   'alpha',0,   'offset',0,    'qlim',[-720  720]*pi/180);

            self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',self.name);            

        end    
    end
end

