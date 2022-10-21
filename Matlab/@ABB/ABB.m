classdef ABB < handle
    properties
        %> Robot model
        model;
        
        %> workspace
        workspace = [-1 1 -1 1 -0.2 1.1];   
      
    end
    
    methods%% Class for custom ABB robot simulation
        function self = ABB(toolModelAndTCPFilenames)
            if 0 < nargin
                if length(toolModelAndTCPFilenames) ~= 2
                    error('Please pass a cell with two strings, toolModelFilename and toolCenterPointFilename');
                end
                self.toolModelFilename = toolModelAndTCPFilenames{1};
                self.toolParametersFilenamure = toolModelAndTCPFilenames{2};
            end
            
            self.GetABBRobot();
            %self.PlotAndColourRobot();%robot,workspace);

            drawnow
        end

        %% GetABBRobot
        % Given a name (optional), create and return the robot model
        function GetABBRobot(self)
            pause(0.001);
            name = ['ABB_',datestr(now,'yyyymmddTHHMMSSFFF')];
            L1 = Link('d',0.1,'a',0,'alpha',pi/2,'qlim',[-pi pi])
            L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
            L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
            L4 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
            L5 = Link('d',0,'a',0.1,'alpha',pi/2,'qlim',[-pi pi])
            L6 = Link('d',0.2,'a',0,'alpha',0,'qlim',[-pi pi])
             
            self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',name);
        end

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        %{
        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['ur3link_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>                
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
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
        %}
    end
end
