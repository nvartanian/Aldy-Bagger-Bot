classdef RobotBaseClass < handle
    properties
        %> Robot model
        model;
        
        % Name of the robot (to be copied into model.name)
        name;

        %> workspace
        workspace = [-2 2 -2 2 -0.3 2];   
        
        %> Flag to indicate if gripper is used
        useGripper = false; 

        %> The first few letters or the name of ply files in the same directory
        % plyFileNameStem = "";
    end

    properties (Access = private)
        delaySecondsForInnerAnimation = 0.2;
        stepsForInnerAnimation = 20;
    end
    
    methods
    
%% General class for multiDOF robot simulation
        function self = RobotBaseClass()
            % This is intentionally left empty. Implement the class
            % constructor in the inhereting class.
            pause(0.001);
            try 
                self.name = [self.plyFileNameStem,datestr(now,'yyyymmddTHHMMSSFFF')];
            catch
                self.name = ['RobotBaseClass',datestr(now,'yyyymmddTHHMMSSFFF')];
                warning(['Please include a variable called plyFileNameStem in your inherreting class. For now the robot is named: ',self.name])                
            end
        end

%% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and colour them in if data is available 
        function PlotAndColourRobot(self)
            for linkIndex = 0:self.model.n
                if self.useGripper && linkIndex == self.model.n
                    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread([self.plyFileNameStem,'_J',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
                else
                    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread([self.plyFileNameStem,'_J',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                end
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end
        
            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;
        
            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                vertexColours = [0.5,0.5,0.5]; % Default if no colours in plyData
                try 
                     vertexColours = [plyData{linkIndex+1}.vertex.red ...
                                     , plyData{linkIndex+1}.vertex.green ...
                                     , plyData{linkIndex+1}.vertex.blue]/255;
                    
                catch ME_1
                    disp(ME_1);
                    display('No vertex colours in plyData');
                    try 
                         vertexColours = [plyData{linkIndex+1}.face.red ...
                                     , plyData{linkIndex+1}.face.green ...
                                     , plyData{linkIndex+1}.face.blue]/255;
                    catch ME_1
                        disp(ME_1);
                        display('Also, no face colours in plyData, so using a default colour');
                    end
                end
                h.link(linkIndex+1).Children.FaceVertexCData = vertexColours;
                h.link(linkIndex+1).Children.FaceColor = 'interp';
            end
        end        
    end

    methods (Hidden)
%% TestMoveJoints      
        % Simple test move of the joints
        function TestMoveJoints(self)
            qPath = jtraj(self.model.qlim(:,1)',self.model.qlim(:,2)',50);
            initialDelay = self.model.delay;
            self.model.delay = self.delaySecondsForInnerAnimation;
            self.model.animate(qPath);
            self.model.delay = initialDelay;
        end

%% TestMoveBase
        % Simple test move of the base
        function TestMoveBase(self)
            startBaseTR = self.model.base;
            self.MoveBaseToTR(startBaseTR,transl(-1,-1,0));
            self.MoveBaseToTR(transl(-1,-1,0),transl(1,1,0));
            self.MoveBaseToTR(transl(1,1,0),startBaseTR);
        end

%% MoveBaseToTR 
        % move robot base through a ctraj generated path
        function MoveBaseToTR(self,startTR, endTR)
            trPath = ctraj(startTR,endTR,self.stepsForInnerAnimation);
            for i = 1:size(trPath,3)
                self.model.base = trPath(:,:,i);
                self.model.animate(self.model.getpos);
                pause(self.delaySecondsForInnerAnimation);
            end
        end
    end
end