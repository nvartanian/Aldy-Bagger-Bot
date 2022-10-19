%% Sawyer robot
classdef Sawyer < handle
    properties (Constant)
        %> Sawyer joint count
        jointCount = 7;
        
        %> Default base location
        defaultBasePose = transl(0,0,0); %* rpy2tr(0, 0, pi/2)
        textboxPosition = [0 0.9 0.1 0.1];
        
        %> Joint angles (all zero)
        qz = zeros(1,Sawyer.jointCount);
        qDefault = [0,-pi/2,0,-pi/2,0,pi/2,0];
        qStraightUp = [0,-pi/2,0,0,0,0,0];
        
        %> When to use DLS
        manipMeasureEpsilon = 0.01;    % was 0.001 for a long time
        
        %> The zone close to the limits where the jabocian is changed to
        %reduce the chance of reaching a singularity
        jointLimitSafety = 20 * pi/180;
        
        %> So we can set back to the default
        defaultRadsPerStep = 5 * pi/180;
    end
    
    properties
        %> Robot models in cell array
        model
        
        %> Importance weighting of joints
        jointWeight = Sawyer.jointCount:-1:1;
        
        %> Text status message plot handle for robot in cells
        text_h
        
        %> Holds the figure and axis (may not be needed)
        fig_h;
        axis_h;
        
        %> Define the boundaries of the workspace
        workspace = [-2 2 -2.5 2.5 -0.7 1.5];
        
        %> Define robot scale
        scale = 0.2;
        
        %> The robot handle in the figure for quick robot plot animating
        robotObjHandle;
        
        %> Holds all data from ply file about robot so it only has to be read once
        sawyerVertexData;
        sawyerFaceData;
        sawyerPlyData;
        
        %> For the output messages and status display
        statusPrecision = 2;
        
        %> Holds the plot handle for the point cloud of the robot
        robotWorkspaceCloudPlot_h;
        
        %> To computer the default number of steps when creating a move
        radsPerStep = Sawyer.defaultRadsPerStep;
    end
    
    properties (Access = protected)
        %> Update the textbox
        updateTextBox = true;
    end
    
    methods
        function self = Sawyer( axis_h )
            if nargin == 1 && strcmp(get(axis_h,'type'),'axes')
                self.axis_h = axis_h;
                self.fig_h = get(curr_axis,'Parent');
            else
                set(0,'DefaultFigureWindowStyle','docked')
            end
            
            self.CreateRobot();
            self.UpdateRobotObjHandle();
        end
        
        %% UpdateRobotObjHandle
        function UpdateRobotObjHandle(self)
            self.robotObjHandle = findobj('Tag', self.model.name);
        end
        
        %% ShowTextStatusBox
        function ShowTextStatusBox(self,value)
            if value && isempty(self.text_h);
                self.text_h = annotation(gcf,'Textbox', self.textboxPosition,'BackgroundColor','white','FaceAlpha',.5);
                set(self.text_h,'String',[self.model.name,' loaded'],'Fontsize',9);
            else
                try  %#ok<TRYNC>
                    delete(self.text_h);
                end
                self.text_h = [];
            end
        end
        
        %% CreateRobot
        function CreateRobot(self)
            pause(0.001);
            name = ['Sawyer@',datestr(now,'yyyymmddTHHMMSSFFF')];
            self.model = self.GetModel(name);
            self.model.base = self.defaultBasePose;
            self.ShowTextStatusBox(true);
            self.PlotAndColourRobot();
            hold on;
        end
        
        %%Partial jacob
            function partialj = partialjacob (self, q)
            n = size(self.model.links,2); % no. link
            dJdq = zeros(6,n,n);
            
            
            [ee ,all] = self.model.fkine(q);
            
            r(:,1) = ee(1:3,4);
            a(:,1) = self.model.base(1:3,3); % this needs fixing
            
            for i = 2:n
                
                r(:,i) = ee(1:3,4)-all(1:3,4,i-1);
                a(:,i) = all(1:3,3,i-1);
                
            end
            
            for i = 1:n
                for j = 1:n
                    if j < i
                        dJdq(:,i,j) = [cross(a(:,j),cross(a(:,i),r(:,i)))
                            cross(a(:,j),a(:,i))]; 
                    elseif j == i
                        dJdq(1:3,i,j) = cross(a(:,j),cross(a(:,i),r(:,i))); %last 3 rows are already zero
                    else
                        dJdq(1:3,i,j) = cross(a(:,i),cross(a(:,j),r(:,j)));
                    end
                    
                end
            end
            
            partialj = dJdq;
        end
        
        
                %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourRobot(self)%robot,workspace)
            mpath = strrep(which(mfilename),[mfilename '.m'],'');

            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread([mpath,'ply/J',num2str(linkIndex),'.ply'],'tri');
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','notiles'); %,'workspace',self.workspace
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end      
    end
    methods (Static)
        %% GetModel
        % Given a name (optional), create and return a Sawyer robot model
        function robot = GetModel(name)
            if nargin < 1
                % Create a unique name (ms timestamp after 1ms pause)
                pause(0.001);
                name = ['SawyerRobot',datestr(now,'yyyymmddTHHMMSSFFF')];
            end
            % Create Sawyer Arm - Joint angles obtained from: http://mfg.rethinkrobotics.com/wiki/Robot_Hardware#tab=Sawyer
            L1 = Link('d',0.317,'a',0.081,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-175),deg2rad(175)], ...
                        'm', 5.3213);
            L2 = Link('d',0.1925,'a',0,'alpha',pi/2,'offset',pi/2,'qlim',[deg2rad(-219),deg2rad(131)], ...
                        'm', 4.505);
            L3 = Link('d',0.4,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-175),deg2rad(175)], ...
                        'm', 1.745);
            L4 = Link('d',-0.1685,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-175),deg2rad(175)], ...
                        'm', 2.5097);
            L5 = Link('d',0.4,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-175),deg2rad(175)], ...
                        'm', 1.1136);
            L6 = Link('d',0.1363,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-175),deg2rad(175)], ...
                        'm', 1.5625);
            L7 = Link('d',0.13375,'a',0,'alpha',0,'offset',-pi/2,'qlim',[deg2rad(-270),deg2rad(270)], ...
                        'm', 0.3292);
            robot = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name',name);
        end
      
        
    
    end    
end