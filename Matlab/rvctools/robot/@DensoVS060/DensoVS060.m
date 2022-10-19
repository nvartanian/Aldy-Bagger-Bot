classdef DensoVS060 < RobotBaseClass
    %% DensoVS060
    % This class is based on the DensoVS060. 
    % URL: https://www.denso-wave.com/en/robot/product/five-six/vp.html
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)  
        plyFileNameStem = 'DensoVS060';
    end
    methods (Access = public) %% Class for DensoVS060 robot simulation
        %% Define robot Function  
        function self = DensoVS060()
            self.CreateModel();
            self.PlotAndColourRobot();            
        end

        %% Create the robot model
        function CreateModel(self)       
            L1=Link('alpha',pi/2,'a',0, 'd',0.148, 'offset',pi,'qlim',[deg2rad(-170), deg2rad(170)] );
            L2=Link('alpha',0,'a',0.305, 'd',0, 'offset',pi/2,'qlim',[deg2rad(-120), deg2rad(120)]);
            L3=Link('alpha',-pi/2,'a',0, 'd',0, 'offset',0,'qlim',[deg2rad(-125), deg2rad(155)]);
            L4=Link('alpha',pi/2,'a',0, 'd',0.3, 'offset',0,'qlim',[deg2rad(-270), deg2rad(270)]);
            L5=Link('alpha',-pi/2,'a',0, 'd',0, 'offset',pi/2,'qlim',[deg2rad(-120), deg2rad(120)]);
            L6=Link('alpha',0,'a',0, 'd',0.06, 'offset',0,'qlim',[deg2rad(-360), deg2rad(360)]);

            self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',self.name);            
        end    
    end
end