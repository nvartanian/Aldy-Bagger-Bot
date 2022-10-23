classdef Bag
	properties
        transform;
		body;
        
        noOfHeavySlots = 2;
        noOfLightSlots = 2;
        heavySlotsOffsets = {transl(0, 0.05, 0.01), transl(0, -0.05, 0.01)}; %hard-coded offsets for bag geometry position relative to base transform
        lightSlotsOffsets = {transl(0, 0.05, 0.11), transl(0, -0.05, 0.11)}; %hard-coded offsets for bag geometry position relative to base transform
        
        heavySlotsFull = {};
        lightSlotsFull = {};
        
        heavySlotsTransform = {};
        lightSlotsTransform = {};
        onRobot = false;
        handOff = false;
        trajCalculated = false;
        
	end
	methods
        %constructor
        function obj = Bag(name, transform)
            obj.transform = transform;
            L1 = Link('alpha',0,'a',0,'d',0,'offset',0);
            obj.body = SerialLink(L1,'name',name);
            obj.body.base = transform;
            
            [faceData,vertexData] = plyread('crate.ply','tri');
            obj.body.faces = {faceData,[]};
            obj.body.points = {vertexData,[]};
            plot3d(obj.body,0,'delay',0);
            
            for i = 1:obj.noOfHeavySlots
                obj.heavySlotsFull{i} = false;
            end
            for i = 1:obj.noOfLightSlots
                obj.lightSlotsFull{i} = false;
            end
            
            for i = 1:obj.noOfHeavySlots
                obj.heavySlotsTransform{i} = obj.transform * obj.heavySlotsOffsets{i};
                %trplot(obj.transform * obj.heavySlotsOffsets{i});
            end
            for i = 1:obj.noOfLightSlots
                obj.lightSlotsTransform{i} = obj.transform * obj.lightSlotsOffsets{i};
                %trplot(obj.transform * obj.lightSlotsOffsets{i});
            end
        end
        
        function self = moveBag(self, transform)
            self.transform = transform;
            self.body.base = transform;
            self.body.animate(0);
        end
        
	end
end