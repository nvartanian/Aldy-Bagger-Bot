classdef Bag
	properties
        transform;
		body;
        
        noOfHeavySlots = 2;
        noOfLightSlots = 2;
        heavySlotsOffsets = {transl(0, 0.1, 0.01), transl(0, -0.1, 0.01)}; %hard-coded offsets for bag geometry position relative to base transform
        lightSlotsOffsets = {transl(0, 0.1, 0.11), transl(0, -0.1, 0.11)}; %hard-coded offsets for bag geometry position relative to base transform
        
        heavySlotsFull = {};
        lightSlotsFull = {};
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
        end
        
        function self = moveBag(self, transform)
            self.transform = transform;
            self.body.base = transform;
            self.body.animate(0);
        end
        
        function t = nextHeavySlotTransform(self)
            for i = 1:size(self.heavySlotsFull)
                if self.heavySlotsFull{i} == false
                    t = self.transform * self.heavySlotsOffsets{i}; %found an empty slot
                    return;
                end
            end
            t = false; %no heavy slots available
        end
        
        function t = nextLightSlotTransform(self)
            for i = 1:size(self.lightSlotsFull)
                if self.lightSlotsFull{i} == false
                    t = self.transform * self.lightSlotsOffsets{i}; %found an empty slot
                    return;
                end
            end
            t = false; %no light slots available
        end
        
	end
end