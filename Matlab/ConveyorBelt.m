classdef ConveyorBelt
    properties
        %constants
        transform;
        length;
        width;
        
        %variables
        speed = 10; %mm/frame (25hz)
        items = {};
        
        %status
        
    end
    methods
        %constructor
        function obj = ConveyorBelt(transform, length, width)
            obj.transform = transform;
            obj.length = length;
            obj.width = width;
            obj = obj.spawnItem();
        end
        
        function self = spawnItem(self)
            t = self.transform * transl((self.width * rand()) - self.width/2, -self.length/1.1, 0) * trotz(2*pi*rand());
            self.items = [self.items, Item(t)];
        end
        
        function self = stepBeltY(self)
            if self.laserTriggered()
                return %dont step belt
            end
            for i = 1:size(self.items)%step each item
                self.items(i).transform
                self.items(i).transform(2, 4) = self.items(i).transform(2, 4) + self.speed/1000;
                self.items(i) = self.items(i).moveItem(self.items(i).transform);
            end
        end
        
        function t = laserTriggered(self)
            t = false;
            for i = 1:size(self.items)
                if self.items(i).transform(2, 4) > self.transform(2, 4) %check each item on belt
                    t = true;
                    return;
                end
            end
        end
        
    end
end