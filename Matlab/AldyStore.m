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
        
        scanTransform = eye(4) * transl(0.3,1.8,0.85) * trotx(deg2rad(20)); %TODO, work out pose to pass scanner. Trial/error, plot transfrom and check.
        
        %status
        idle = true;
        eStop = false;
        
    end
    methods
        %constructor
        function obj = AldyStore(AldyBaggerBot)
            obj.AldyBaggerBot = AldyBaggerBot;
            obj.AldyBaggerBot.robot.model.base = obj.transform * transl(-0.15, -0.25 + 2, 0.6) * trotz(pi);
            obj.AldyBaggerBot.robot.model.animate(obj.AldyBaggerBot.homePose);
            beltT = obj.transform * transl(0.1, 1.5, 0.75);
            obj.ConveyorBelt = ConveyorBelt(beltT, 3, 0.4); %create ConveyorBelt obj
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
            
            %change default figure view
            view(135,30)
            
        end
        
        function self = stepStore(self)
            self.ConveyorBelt = self.ConveyorBelt.stepBeltY();
            self.AldyBaggerBot = self.AldyBaggerBot.stepRobot();
            
            if self.idle ~= false
                return
            end
            
            for i = 1:size(self.ConveyorBelt.items,2)
                if max(abs(self.AldyBaggerBot.robot.model.fkine(self.AldyBaggerBot.robot.model.getpos) - self.ConveyorBelt.items{i}.transform)) < 0.005
                    self.ConveyorBelt.items{i}.bagged = true;
                end
                if self.ConveyorBelt.items{i}.readyToCollect ~= true
                    %warning('Item %d is not on the belt, continuing to next item', i);
                    continue
                end
%                 if self.ConveyorBelt.items{i}.onBelt ~= true
%                     %warning('Item %d is not on the belt, continuing to next item', i);
%                     continue
%                 end
%                 if self.ConveyorBelt.items{i}.bagged ~= false
%                     %warning('Item %d is already bagged, continuing to next item', i);
%                     continue
%                 end
                if self.AldyBaggerBot.inRange(self.ConveyorBelt.items{i}.transform) ~= true
                    %warning('Item %d is not in range, continuing to next item', i);
                    continue
                end
                if self.ConveyorBelt.items{i}.heavy == true %find a bag spot that can take a heavy item
                    try self.BaggingArea.nextHeavyBag() ~= false
                    catch
                        self = self.generateBaggingPath(self.AldyBaggerBot, self.ConveyorBelt.items{i}, self.BaggingArea.nextHeavyBag());
                        self.ConveyorBelt.items{i}.readyToCollect = true;
                        return
                    end
                else %find a bag spot that can take a light item
                    try self.BaggingArea.nextLightBag() ~= false
                    catch
                        self = self.generateBaggingPath(self.AldyBaggerBot, self.ConveyorBelt.items{i}, self.BaggingArea.nextLightBag());
                        self.ConveyorBelt.items{i}.readyToCollect = true;
                        return
                    end
                end
                %could not find a bag spot, return error TODO
                
            end
        end
        
        function self = generateBaggingPath(self, AldyBaggerBot, Item, Bag)            
            %determine robot approach item pose
            t = Item.transform * transl(0,0,0.2);
            self.AldyBaggerBot = self.AldyBaggerBot.addTraj(t);
            
            %robot grab item
            t = Item.transform * transl(0,0,0.1);
            self.AldyBaggerBot = self.AldyBaggerBot.addTraj(t);
            
            %robot move to safe pose
            t = Item.transform * transl(0,0,0.2);
            self.AldyBaggerBot = self.AldyBaggerBot.addTraj(t);
            
            %robot move past scanner
            t = self.scanTransform;
            self.AldyBaggerBot = self.AldyBaggerBot.addTraj(t);
            
            %robot approach bag
            tb = Bag.nextLightSlotTransform();
            if Item.heavy == false
                tb = Bag.nextHeavySlotTransform();
            end
            t = tb * transl(0, 0, 0.2);
            self.AldyBaggerBot = self.AldyBaggerBot.addTraj(t);
            
            %robot place item
            self.AldyBaggerBot = self.AldyBaggerBot.addTraj(t);
            
            %robot move to home pose
            t = self.AldyBaggerBot.robot.model.fkine(AldyBaggerBot.homePose);
            self.AldyBaggerBot = self.AldyBaggerBot.addTraj(t);
        end
        
    end
end