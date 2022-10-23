classdef Gripper < handle
    properties
        name;
        
        Base;
        LeftFinger;
        RightFinger;
        
        %>
        workspace = [-2 2 -2 2 -0.3 2];
    end
    
    methods%% Class for Gripper robot simulation
        function obj = Gripper(name)
            obj.name = name;
            
            % robot =
            obj.GetBase();
            obj.GetLeftFinger();
            obj.GetRightFinger();
            
            obj.Base.base = eye(4);
            obj.LeftFinger.base = obj.Base.base * transl(-0.038, 0, 0.055) * trotx(deg2rad(-90));
            obj.RightFinger.base = obj.Base.base * transl(0.038, 0, 0.055) * trotx(deg2rad(90));
            
            % robot =
            obj.PlotAndColourRobot();%robot,workspace);
        end
        
        function self = moveGripper(self, newBase, jawQ)
            self.Base.base = newBase;
            self.Base.animate(0);
            self.LeftFinger.base = self.Base.base * transl(-0.038, 0, 0.055) * trotx(deg2rad(-90));
            self.LeftFinger.animate(jawQ(1));
            self.RightFinger.base = self.Base.base * transl(0.038, 0, 0.055) * trotx(deg2rad(90));
            self.RightFinger.animate(jawQ(2));
        end
        
        %% GetGripperRobot
        function self = GetBase(self)
            n = ['Gripper_Base_',self.name];     
            % Create the Left Finger model
            L(1) = Link('d',0,'a',0,'alpha',0,'qlim',0,'offset',0);
            
            self.Base = SerialLink(L,'name',n);
        end
        
        function self = GetLeftFinger(self)
            n = ['Gripper_Left_',self.name];
            % Create the Left Finger model
            L(1) = Link('d',0,'a',0,'alpha',deg2rad(90),'qlim',deg2rad([-360 360]), 'offset',0);
            self.LeftFinger = SerialLink(L,'name',n);
        end
        
        function self = GetRightFinger(self)
            n = ['Gripper_Right_',self.name];
            % Create the Left Finger model
            L(1) = Link('d',0,'a',0,'alpha',deg2rad(-90),'qlim',deg2rad([-360 360]), 'offset',0);
            self.RightFinger = SerialLink(L,'name',n);
        end
        
        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available
        function PlotAndColourRobot(self)%robot,workspace)
            %Load data from .ply files
            
            for linkIndex = 1:self.Base.n
                [ faceData, vertexData, plyData{linkIndex+1} ] = plyread('gripper_Base.ply','tri'); %#ok<AGROW>
                self.Base.faces{linkIndex+1} = faceData;
                self.Base.points{linkIndex+1} = vertexData;
            end
            
            for linkIndex = 1:self.LeftFinger.n
                [ faceData, vertexData, plyData{linkIndex+1} ] = plyread('gripper_Left.ply','tri');
                self.LeftFinger.faces{linkIndex+1} = faceData;
                self.LeftFinger.points{linkIndex+1} = vertexData;
            end
            
            for linkIndex = 1:self.RightFinger.n
                [ faceData, vertexData, plyData{linkIndex+1} ] = plyread('gripper_Right.ply','tri');
                self.RightFinger.faces{linkIndex+1} = faceData;
                self.RightFinger.points{linkIndex+1} = vertexData;
            end
            
            % Display robot
            self.Base.plot3d(zeros(1,self.Base.n),'noarrow','workspace',self.workspace, 'color', {'black'});
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.Base.delay = 0;
            
            self.LeftFinger.plot3d(zeros(1,self.LeftFinger.n),'noarrow','workspace',self.workspace, 'color', {'grey'});
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.LeftFinger.delay = 0;
            
            self.RightFinger.plot3d(zeros(1,self.RightFinger.n),'noarrow','workspace',self.workspace, 'color', {'grey'});
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.RightFinger.delay = 0;
            
            % Try to correctly colour the arm (if colours are in ply file data)
%             handles = findobj('Tag', self.Base.name);
%             h = get(handles,'UserData');
%             try
%                 h.link(1).Children.FaceVertexCData = [plyData{1}.vertex.red ...
%                     , plyData{1}.vertex.green ...
%                     , plyData{1}.vertex.blue]/255;
%                 h.link(1).Children.FaceColor = 'interp';
%             catch ME_1
%                 disp(ME_1);
%             end
%             
%             handles = findobj('Tag', self.LeftFinger.name);
%             h = get(handles,'UserData');
%             try
%                 h.link(2).Children.FaceVertexCData = [plyData{2}.vertex.red ...
%                     , plyData{2}.vertex.green ...
%                     , plyData{2}.vertex.blue]/255;
%                 h.link(2).Children.FaceColor = 'interp';
%             catch ME_1
%                 disp(ME_1);
%             end
%             
%             handles = findobj('Tag', self.RightFinger.name);
%             h = get(handles,'UserData');
%             try
%                 h.link(1).Children.FaceVertexCData = [plyData{1}.vertex.red ...
%                     , plyData{1}.vertex.green ...
%                     , plyData{1}.vertex.blue]/255;
%                 h.link(1).Children.FaceColor = 'interp';
%             catch ME_1
%                 disp(ME_1);
%             end
        end
    end
end