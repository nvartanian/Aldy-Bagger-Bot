classdef AldyStore
    properties
        %constant
        transform = eye(4); %centre of store
        width = 2; %x axis = width
        length = 6; %y axis = length
        height = 2; %z axis = height
        
        AldyBaggerBot;
        ConveyorBelt; 
        BaggingArea; %need class for baggingArea and Bag
        
        scanTransform = eye(4) * transl(0,0,0) * trotx(deg2rad(20)); %TODO, work out pose to pass scanner. Trial/error, plot transfrom and check.
        
        %status
        idle = true;
        eStop = false;
        
    end
    methods
        %constructor
        function obj = AldyStore(AldyBaggerBot)
            obj.AldyBaggerBot = AldyBaggerBot;
            obj.AldyBaggerBot.robot.model.base = obj.transform * transl(-0.25, -0.25 + 2, 0.6);
            obj.AldyBaggerBot.robot.model.animate(obj.AldyBaggerBot.homePose);
            beltT = obj.transform * transl(0.1, 1, 0.75);
            obj.ConveyorBelt = ConveyorBelt(beltT, 2, 0.4); %create ConveyorBelt obj
            obj.BaggingArea = BaggingArea(obj.transform * transl(0.12, 2.3, 0.6) * trotz(deg2rad(30))); %create BaggingArea obj
            
            %setup environment
            % Floor
            floorX = [obj.transform(1, 4) - obj.width/2, obj.transform(1, 4) - obj.width/2;...
                      obj.transform(1, 4) + obj.width/2, obj.transform(1, 4) + obj.width/2];
            floorY = [obj.transform(2, 4) - obj.length/2, obj.transform(2, 4) + obj.length/2;...
                      obj.transform(2, 4) - obj.length/2, obj.transform(2, 4) + obj.length/2];
            floorZ = [obj.transform(3, 4), obj.transform(3, 4);... 
                      obj.transform(3, 4), obj.transform(3, 4)];
            surf(floorX,floorY,floorZ,'CData',imread('whiteConcreteFloor.jpg'),'FaceColor','texturemap');
            axis([obj.transform(1, 4) - obj.width/2, obj.transform(1, 4) + obj.width/2,... 
                  obj.transform(2, 4) - obj.length/2, obj.transform(2, 4) + obj.length/2,...
                  obj.transform(3, 4), obj.transform(3, 4) + obj.height]);

            %Back Wall (to mount camera on)
            wallX = [obj.transform(1, 4) + obj.width/2, obj.transform(1, 4) - obj.width/2;...
                     obj.transform(1, 4) + obj.width/2, obj.transform(1, 4) - obj.width/2];
            wallY = [obj.transform(2, 4) - obj.length/2, obj.transform(2, 4) - obj.length/2;...
                     obj.transform(2, 4) - obj.length/2, obj.transform(2, 4) - obj.length/2];
            wallZ = [obj.transform(3, 4) + obj.height, obj.transform(3, 4) + obj.height;...
                     obj.transform(3, 4), obj.transform(3, 4)];
            surf(wallX,wallY,wallZ,'CData',imread('brickWall.jpg'),'FaceColor','texturemap');

            %Checkout
            checkoutXYZ = [obj.transform(1, 4), obj.transform(2, 4) + 2, obj.transform(3, 4)];
            PlaceObject('AldyCheckout.ply',checkoutXYZ);

            % Safety Features/Enviromental Objects
            % Camera (Mounted on wall)
            %cameraXYZ = [-obj.width/1.5, -obj.length, obj.height/1.5];
            %PlaceObject('cctv.ply',cameraXYZ);

            % Fire Extinguisher (Mounted on wall in centre)
            %fireXYZ = [-obj.width/2, -obj.length + 0.2, 0];
            %PlaceObject('fire.ply',fireXYZ);

            % E-Stop (On Table)
            %eStopXYZ = [tableXYZ(1) - tableRad * 0.9,tableXYZ(2) + tableRad * 0.9,tableXYZ(3) + 0.445];
            %PlaceObject('eStop.ply',eStopXYZ);

            %safety features
            %e-stop
            
            %light curtain
            
        end
        
        function self = stepStore(self)
            self.ConveyorBelt = self.ConveyorBelt.stepBeltY();
            self.AldyBaggerBot.stepRobot();
            
            if self.idle ~= false
                return
            end
            
            for i = 1:size(self.ConveyorBelt.items)
                if self.ConveyorBelt.items{i}.onBelt ~= true
                    continue
                end
                if self.ConveyorBelt.items{i}.bagged ~= false
                    continue
                end
                if self.inRange(self.ConveyorBelt.items{i}.transform) ~= true
                    continue
                end
                if self.ConveyorBelt.items{i}.heavy == true %find a bag spot that can take a heavy item
                    if self.BaggingArea.nextHeavyBag() ~= false
                        self = self.generateBaggingPath(self.AldyBaggerBot, self.ConveyorBelt.items{i}, self.BaggingArea.nextHeavyBag());
                        self.ConveyorBelt.items{i}.bagged = true;
                        return
                    end
                else %find a bag spot that can take a light item
                    if self.BaggingArea.nextLightBag() ~= false
                        self = self.generateBaggingPath(self.AldyBaggerBot, self.ConveyorBelt.items{i}, self.BaggingArea.nextLightBag());
                        self.ConveyorBelt.items{i}.bagged = true;
                        return
                    end
                end
                %could not find a bag spot, return error TODO
                
            end
        end
        
        function r = inRange(self, transform)
            r = false;
            %TODO: determine if in range, return true if in range of robot
        end
        
        function self = generateBaggingPath(self, AldyBaggerBot, Item, Bag)
            t0 = AldyBaggerBot.robot.model.ikine(AldyBaggerBot.homePose);
            
            %determine robot approach item pose
            t1 = Item.transform * transl(0,0,0.2);
            AldyBaggerBot.path = [AldyBaggerBot.path, AldyBaggerBot.calculateTraj(t0, t1)]; %add to path
            
            %robot grab item
            t2 = Item.transform * transl(0,0,0.1);
            AldyBaggerBot.path = [AldyBaggerBot.path, AldyBaggerBot.calculateTraj(t1, t2)]; %add to path
            
            %robot move to safe pose
            t3 = Item.transform * transl(0,0,0.2);
            AldyBaggerBot.path = [AldyBaggerBot.path, AldyBaggerBot.calculateTraj(t2, t3)]; %add to path
            
            %robot move past scanner
            t4 = self.scanTransform;
            AldyBaggerBot.path = [AldyBaggerBot.path, AldyBaggerBot.calculateTraj(t3, t4)]; %add to path
            
            %robot approach bag
            t6 = Bag.nextLightSlotTransform();
            if Item.heavy == false
                t6 = Bag.nextHeavySlotTransform();
            end
            t5 = t6 * transl(0, 0, 0.2);
            AldyBaggerBot.path = [AldyBaggerBot.path, AldyBaggerBot.calculateTraj(t4, t5)]; %add to path
            
            %robot place item
            AldyBaggerBot.path = [AldyBaggerBot.path, AldyBaggerBot.calculateTraj(t5, t6)]; %add to path
            
            %robot move to home pose
            AldyBaggerBot.path = [AldyBaggerBot.path, AldyBaggerBot.calculateTraj(t6, t0)]; %add to path
        end
        
    end
end