classdef Item
	properties
		length = rand() * 0.1 + 0.1; %0.1 to 0.2
        width = rand() * 0.1 + 0.1; %0.1 to 0.2
        height = rand() * 0.1 + 0.1; %0.1 to 0.2
        
        transform;
		body;
        onBelt = false;
        bagged = false;

		% optional properties
		heavy = true; %for checking/sorting
		type %for sorting
	end
	methods
        %constructor
        function obj = Item(name, transform)
            obj.transform = transform;
            L1 = Link('alpha',0,'a',0,'d',0,'offset',0);
            obj.body = SerialLink(L1,'name',name);
            obj.body.base = transform;
            
            [faceData,vertexData] = plyread('HalfSizedRedGreenBrick.ply','tri');
            obj.body.faces = {faceData,[]};
            obj.body.points = {vertexData,[]};
            plot3d(obj.body,0,'delay',0);
        end
        
        function self = moveItem(self, transform)
            self.transform = transform;
            self.body.base = transform;
            self.body.animate(0);
        end
 
	end
end