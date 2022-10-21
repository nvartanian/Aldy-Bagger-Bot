classdef ConveyorBelt
    properties
        %constants
        transform;
        length;
        width;
        maxItemCount = 10; %will spawn all of these in at the start for performance improvement
        averageTicksBetweenSpawns = 25; %items spawn randowmly +/- 50% of this spawn rate
        
        %variables
        speed = 10; %mm/frame (25hz)   
        items = {};
        ticksSinceLastSpawn = 0;
        ticksUntilNextSpawn = 0;  %randomly generated after each spawn based on AverageTicksBetweenSpawns +/- 50%
        
        %status
        updateLaser = true;
    end
    methods
        %constructor
        function obj = ConveyorBelt(transform, length, width)
            obj.transform = transform;
            obj.length = length;
            obj.width = width;
            obj.ticksUntilNextSpawn = obj.averageTicksBetweenSpawns * (rand() + 0.5);
            
            %spawn all items at start, rather than one at a time. This
            %reduces stuttering and view reseting if items are spawned in
            %real time. Items are hidden underneath the conveyor belt until
            %"spawned". 
            obj = obj.spawnItems();
        end
        
        %methods
        function self = stepBeltY(self)  
            %check if laser is triggered
            if self.laserTriggered() == true
                if self.updateLaser == true
                    self = self.laserUpdate();
                    self.updateLaser = false;
                end
                return %stop item spawning, dont step belt
            end
            self.updateLaser = true;
            
            %spawn item? - actually enable and move up onto belt
            self.ticksSinceLastSpawn = self.ticksSinceLastSpawn + 1;
            if self.ticksSinceLastSpawn > self.ticksUntilNextSpawn
                for i = 1:self.maxItemCount
                    if self.items{i}.onBelt == false %find first not on belt
                        self.items{i}.onBelt = true;
                        self.items{i}.transform(3, 4) = self.items{i}.transform(3, 4) + 0.5; %move up 0.5m, onto belt
                        self.items{i} = self.items{i}.moveItem(self.items{i}.transform);
                        self.ticksSinceLastSpawn = 0;
                        self.ticksUntilNextSpawn = self.averageTicksBetweenSpawns * (rand() + 0.5);
                        break;
                    end
                    %all items on belt
                end
            end
            
            %step each item
            for i = 1:self.maxItemCount
                if self.items{i}.bagged == true
                   continue 
                end
                if self.items{i}.onRobot == true
                   continue 
                end
                if self.items{i}.onBelt == true
                    self.items{i}.transform(2, 4) = self.items{i}.transform(2, 4) + self.speed/1000;
                    self.items{i} = self.items{i}.moveItem(self.items{i}.transform);
                end
            end
        end
        
        function t = laserTriggered(self)
            t = false;
            for i = 1:size(self.items, 2)
                if self.items{i}.bagged == true
                    continue
                end
                if self.items{i}.transform(2, 4) > self.transform(2, 4)
                    if self.items{i}.transform(3, 4) < self.transform(3, 4) + 0.5 %check each item on belt
                        t = true;
                        return;
                    end
                end
            end
        end
        
        function self = laserUpdate(self)
            for i = 1:size(self.items, 2)
                if self.items{i}.bagged == true
                    continue
                end
                if self.items{i}.transform(2, 4) > self.transform(2, 4) %check each item on belt
                    self.items{i}.readyToCollect = true;
                    return;
                end
            end
        end
        
        function self = spawnItems(self)
            for i = 1:self.maxItemCount
                t = self.transform * transl((self.width * rand()) - self.width/2, -self.length/1.1, -0.5) * trotz(2*pi*rand());
                if rand() > 0.5
                    self.items{i} = Item(['item',num2str(i)], t, true);
                else
                    self.items{i} = Item(['item',num2str(i)], t, false);
                end
            end
        end
        
    end
end