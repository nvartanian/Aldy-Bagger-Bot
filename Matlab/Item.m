classdef Item
	properties
		length = rand() * 0.1 + 0.1; %0.1 to 0.2
        width = rand() * 0.1 + 0.1; %0.1 to 0.2
        height = rand() * 0.1 + 0.1; %0.1 to 0.2
        
        transform;
		body;
        onBelt = false; %goes true when item is enabled
        readyToCollect = false; %goes true when laser is triggered
        trajCalculated = false; %goes true when traj is calculated
        bagged = false; %goes true when robot touches item < 0.05m
        onRobot = false;
        
        wayPoints = [];

		% optional properties
		heavy = true; %for checking/sorting
		type %for sorting
	end
	methods
        %constructor
        function obj = Item(name, transform, isHeavy)
            obj.transform = transform;
            L1 = Link('alpha',0,'a',0,'d',0,'offset',0);
            obj.body = SerialLink(L1,'name',name);
            obj.body.base = transform;
            if isHeavy == true
                if rand() > 0.5
                    [faceData, vertexData, plyData] = plyread('cannedTomato.ply','tri');
                else
                    [faceData, vertexData, plyData] = plyread('HalfSizedRedGreenBrick.ply','tri');
                end
            else
                obj.heavy = false;
                if rand() > 0.5
                    [faceData, vertexData, plyData] = plyread('HalfSizedRedGreenBrick.ply','tri');
                else
                    [faceData, vertexData, plyData] = plyread('HalfSizedRedGreenBrick.ply','tri');
                end
            end
            
            obj.body.faces = {faceData,[]};
            obj.body.points = {vertexData,[]};
            plot3d(obj.body,0,'delay',0);
            handles = findobj('Tag', obj.body.name);
            h = get(handles,'UserData');
            try
                h.link(1).Children.FaceVertexCData = [plyData.vertex.red ...
                                                               ,plyData.vertex.green ...
                                                               ,plyData.vertex.blue]/255;
                h.link(1).Children.FaceColor = 'interp';
            catch ME_1
                disp(ME_1);
            end
        end
        
        function self = moveItem(self, transform)
            self.transform = transform;
            self.body.base = transform;
            self.body.animate(0);
        end
 
	end
end