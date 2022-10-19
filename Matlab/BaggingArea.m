classdef BaggingArea
	properties
        transform;
        
        noOfBags = 2;
        bagOffsets = {transl(0.12, 0, 0), transl(-0.12, 0, 0)}; %hard-coded offsets for bag geometry position relative to base transform
        
        bags = {};
	end
	methods
        %constructor
        function obj = BaggingArea(transform)
            obj.transform = transform;
            for i = 1:obj.noOfBags
                t = obj.transform * obj.bagOffsets{i};
                obj.bags{i} = Bag(['bag',num2str(i)],t);
            end
        end
        
        function bag = nextHeavyBag(self)
            for i = 1:size(self.bags)
                for j = 1:size(self.bags{i})
                    if self.bags{i}.heavySlotsFull{j} == false
                        bag = self.bags{i}; %found an empty slot
                        return;
                    end
                end
            end
            bag = false;
        end
        
        function bag = nextLightBag(self)
            for i = 1:size(self.bags)
                for j = 1:size(self.bags{i})
                    if self.bags{i}.lightSlotsFull{j} == false
                        bag = self.bags{i}; %found an empty slot
                        return;
                    end
                end
            end
            bag = false;
        end
        
        function self = changeBag(self, bagIndex)
            self.bags{bagIndex}; %TODO     
        end
	end
end