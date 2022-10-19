classdef MotomanHC10DTP < RobotBaseClass
    %% MotomanHC10DTP
    % This class is based on the MotomanHC10DTP. 
    % URL: https://www.yaskawa.eu.com/products/robots/collaborative/productdetail/product/hc10dtp_17024
    % 
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'MotomanHC10DTP';
    end

    methods (Access = public) 
        %% Define robot Function  
        function self = MotomanHC10DTP()
            self.CreateModel();
            self.PlotAndColourRobot();
        end

        %% Create the robot model
        function CreateModel(self)       
            L1 = Link('d',0.275,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            L2 = Link('d',0,'a',0.7,'alpha',0,'offset',pi/2,'qlim',[deg2rad(-90),deg2rad(90)]);
            L3 = Link('d',0,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
            L4 = Link('d',0.5,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            L5 = Link('d',0.162,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            L6 = Link('d',0.170,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            
            self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',self.name);            
        end    
    end
end