classdef Person
	properties
        transform;
		body;
	end
	methods
        %constructor
        function obj = Person(name, transform)
            obj.transform = transform;
            L1 = Link('alpha',0,'a',0,'d',0,'offset',0);
            obj.body = SerialLink(L1,'name',name);
            obj.body.base = transform;
            
            [faceData, vertexData, plyData] = plyread('person.ply','tri');
            
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