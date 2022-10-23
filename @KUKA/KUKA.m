classdef KUKA < handle
    properties
        %> Robot model
        model;
        
        %> workspace
        workspace = [-3 3 -3 3 -0.2 3];   
      
    end
    
    methods%% Class for custom KUKA robot simulation
        function self = KUKA(toolModelAndTCPFilenames)
            if 0 < nargin
                if length(toolModelAndTCPFilenames) ~= 2
                    error('Please pass a cell with two strings, toolModelFilename and toolCenterPointFilename');
                end
                self.toolModelFilename = toolModelAndTCPFilenames{1};
                self.toolParametersFilenamure = toolModelAndTCPFilenames{2};
            end
            
            self.GetKUKARobot();
            self.PlotAndColourRobot();%robot,workspace);

            drawnow
        end

        %% GetKUKARobot
        % Given a name (optional), create and return the robot model
        function GetKUKARobot(self)
            pause(0.001);
                   
            name = ['KUKA_',datestr(now,'yyyymmddTHHMMSSFFF')];
            L1 = Link('d',0.17,'a',0,'alpha',pi/2, 'offset',0,   'qlim',deg2rad([-170 170]));
            L2 = Link('d',0,   'a',0,'alpha',-pi/2,   'offset',0,'qlim',deg2rad([-120 120]));
            L3 = Link('d',0.4, 'a',0,'alpha',pi/2,    'offset',0,   'qlim',deg2rad([-170 170]));
            L4 = Link('d',0,   'a',0,'alpha',-pi/2,  'offset',0,   'qlim',deg2rad([-120 120]));
            L5 = Link('d',0.4, 'a',0,'alpha',pi/2,    'offset',0,   'qlim',deg2rad([-170 170]));
            L6 = Link('d',0,   'a',0,'alpha',-pi/2,    'offset',0,   'qlim',deg2rad([-120 120]));
            L7 = Link('d',0.05,'a',0,'alpha',0,    'offset',0,   'qlim',deg2rad([-175 175]));

            self.model = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name',name);
          
            
        end

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['KUKAlink_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>                
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            %Try to correctly colour the arm (if colours are in ply file data)
            
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
end
