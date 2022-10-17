classdef ConveyorBelt
    properties
        %constants
        transform;
        length;
        width;
        
        %variables
        speed = 1; %mm/frame (100hz)
        items = {};
        
        %status
        
    end
    methods
        %constructor
        function obj = ConveyorBelt(transform, length, width)
            obj.transform = transform;
            obj.length = length;
            obj.width = width;
        end
        
        function self = spawnItem(self)
            pose = self.transform * transl(self.length/2.1, (self.width * rand()) - self.width/2, 0) * trotz(2*pi*rand());
            self.items = [self.items, Item(pose)];
        end
        
        function stepBeltX(self)
            if self.laserTriggered() 
                return %dont step belt
            end
            for i = 1:size(self.items) %step each item
                self.items(i).transform = self.items(i).transform * transl(-self.speed/1000, 0, 0);
            end 
        end
        
        function t = laserTriggered(self)
            t = false;
            for i = 1:size(self.items)
                if collision %check each item on belt
                    t = true;
                    return;
                end
            end
        end
                
    end
end