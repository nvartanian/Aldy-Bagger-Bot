classdef AldyStore
    properties
        %constant
        transform = eye(4); %centre of store
        width = 3; %x axis = width
        length = 6; %y axis = length
        height = 2; %z axis = height
        
        AldyBaggerBotUR3;
        AldyBaggerBotKUKA;
        ConveyorBelt; 
        BaggingArea; %need class for baggingArea and Bag
        
        Person;
        lightCurtainEnabled = false;
        lightCurtainX = 0.9;
        lightCurtainY = 2.5;
        
        scanTransform = eye(4) * transl(0.3,1.8,1) * trotx(deg2rad(180)); %TODO, work out pose to pass scanner. Trial/error, plot transfrom and check.
        
        %status
        idle = true;
        eStop = false;
        
    end
    methods
        %constructor
        function obj = AldyStore(AldyBaggerBot6DOF,AldyBaggerBot7DOF)
            obj.AldyBaggerBotUR3 = AldyBaggerBot6DOF;
            obj.AldyBaggerBotUR3.robot.model.base = obj.transform * transl(0, 1.75, 0.75) * trotz(pi);
            obj.AldyBaggerBotUR3.robot.model.animate(obj.AldyBaggerBotUR3.homePose);
            
            obj.AldyBaggerBotKUKA = AldyBaggerBot7DOF; % add kuka bot
            obj.AldyBaggerBotKUKA.robot.model.base = obj.transform * transl(-0.3, 0.5, 0.8) * trotz(pi);
            obj.AldyBaggerBotKUKA.robot.model.animate(obj.AldyBaggerBotKUKA.homePose);
            
            beltT = obj.transform * transl(0.11, 1.5, 0.75);
            obj.ConveyorBelt = ConveyorBelt(beltT, 3, 0.2); %create ConveyorBelt obj
            obj.BaggingArea = BaggingArea(obj.transform * transl(0.12, 2.1, 0.75) * trotz(deg2rad(0))); %create BaggingArea obj
            
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
            PlaceObject('AldyCheckoutV2.ply',checkoutXYZ);

            % Add Person
            personT = obj.transform * transl(1, 2, 0);
            obj.Person = Person('person1', personT);
            
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
            if self.lightCurtainEnabled == true
                if self.Person.body.base(1, 4) < self.lightCurtainX
                    if self.Person.body.base(2, 4) < self.lightCurtainY
                        self.eStop = true; %trigger E-stop.
                        return;
                    end
                end
            end
            
            %collision detection checking
                
            if self.idle ~= false
                return
            end
            self.AldyBaggerBotUR3 = self.AldyBaggerBotUR3.stepRobot();
            self.AldyBaggerBotKUKA = self.AldyBaggerBotKUKA.stepRobot();
            
            for i = 1:size(self.ConveyorBelt.items,2)
                if self.ConveyorBelt.items{i}.bagged ~= false
                    %warning('Item %d is already bagged, continuing to next item', i);
                    continue
                end
                if self.ConveyorBelt.items{i}.readyToCollect ~= true
                    %warning('Item %d is not on the belt, continuing to next item', i);
                    continue
                end
                if self.ConveyorBelt.items{i}.onRobot == true
                    %move brick to robot
                    %check place brick
                    if max(max(abs(self.AldyBaggerBotUR3.robot.model.fkine(self.AldyBaggerBotUR3.robot.model.getpos) - self.ConveyorBelt.items{i}.wayPoints{6}))) < 0.005
                        self.ConveyorBelt.items{i}.bagged = true;
                        %open gripper
                        continue
                    end
                    %not at bag yet, keep brick on robot
                    self.ConveyorBelt.items{i} = self.ConveyorBelt.items{i}.moveItem(self.AldyBaggerBotUR3.robot.model.fkine(self.AldyBaggerBotUR3.robot.model.getpos) * transl(0,0,0.1) * trotx(deg2rad(180)));
                    continue
                end
                if self.ConveyorBelt.items{i}.trajCalculated ~= false
                    %traj calculated but not yet on robot
                    %check collect brick
                    if max(max(abs(self.AldyBaggerBotUR3.robot.model.fkine(self.AldyBaggerBotUR3.robot.model.getpos) - self.ConveyorBelt.items{i}.wayPoints{1}))) < 0.005
                        %start closing gripper, use same number of steps as
                        %robot traj calculation, add a closing traj to
                        %gripper. need a step gripper --> inside
                        %aldybaggerbot object
                    end
                    if max(max(abs(self.AldyBaggerBotUR3.robot.model.fkine(self.AldyBaggerBotUR3.robot.model.getpos) - self.ConveyorBelt.items{i}.wayPoints{2}))) < 0.005
                        self.ConveyorBelt.items{i}.onRobot = true;
                        %gripper should be closed
                    end
                    continue
                end
                
%                 if self.ConveyorBelt.items{i}.onBelt ~= true
%                     %warning('Item %d is not on the belt, continuing to next item', i);
%                     continue
%                 end

                if self.AldyBaggerBotUR3.inRange(self.ConveyorBelt.items{i}.transform * transl(0,0,0.2) * trotx(deg2rad(180))) ~= true
                    %warning('Item %d is not in range, continuing to next item', i);
                    continue
                end
                if self.ConveyorBelt.items{i}.heavy == true %find a bag spot that can take a heavy item
                    [bagIndex, slotIndex] = self.BaggingArea.nextHeavyBag();
                    if isa(bagIndex, 'bool') ~= true
                        self.BaggingArea.bags{bagIndex}.heavySlotsFull{slotIndex} = true;
                        self = self.generateBaggingPath(i, self.BaggingArea.bags{bagIndex}.heavySlotsTransform{slotIndex}*transl(0,0,0.1));
                        self.ConveyorBelt.items{i}.trajCalculated = true;
                        return
                    end
                else %find a bag spot that can take a light item
                    [bagIndex, slotIndex] = self.BaggingArea.nextLightBag();
                    if isa(bagIndex, 'bool') ~= true
                        self.BaggingArea.bags{bagIndex}.lightSlotsFull{slotIndex} = true;
                        self = self.generateBaggingPath(self.AldyBaggerBotUR3, i, self.BaggingArea.bags{bagIndex}.lightSlotsTransform{slotIndex}*transl(0,0,0.1));
                        self.ConveyorBelt.items{i}.trajCalculated = true;
                        return
                    end
                end
                %could not find a bag spot, return error TODO
                
            end
        end
        
        function self = generateBaggingPath(self, itemIndex, slotTransform)            
            %determine robot approach item pose
            self.ConveyorBelt.items{itemIndex}.wayPoints{1} = self.ConveyorBelt.items{itemIndex}.transform * transl(0,0,0.15) * trotx(deg2rad(180));
            
            %robot grab item
            self.ConveyorBelt.items{itemIndex}.wayPoints{2} = self.ConveyorBelt.items{itemIndex}.transform * transl(0,0,0.1) * trotx(deg2rad(180));
            
            %robot move to safe pose
            t = self.ConveyorBelt.items{itemIndex}.transform * transl(0,0,0.2) * trotx(deg2rad(180));
            t(1:3, 1:3) = self.scanTransform(1:3, 1:3);
            self.ConveyorBelt.items{itemIndex}.wayPoints{3} = t;
            
            %robot move past scanner
            self.ConveyorBelt.items{itemIndex}.wayPoints{4} = self.scanTransform;
            
            %robot approach bag
            self.ConveyorBelt.items{itemIndex}.wayPoints{5} = slotTransform * transl(0, 0, 0.2) * trotx(deg2rad(180)) * trotz(deg2rad(90));
            
            %robot place item
            self.ConveyorBelt.items{itemIndex}.wayPoints{6} = slotTransform * trotx(deg2rad(180)) * trotz(deg2rad(90));
            
            %robot move away from bag
            self.ConveyorBelt.items{itemIndex}.wayPoints{7} = slotTransform * transl(0, 0, 0.2) * trotx(deg2rad(180)) * trotz(deg2rad(90));
            
            %robot move to home pose
            %self.ConveyorBelt.items{itemIndex}.wayPoints{8} = self.AldyBaggerBot.robot.model.fkine(AldyBaggerBot.homePose);
            for i = 1:size(self.ConveyorBelt.items{itemIndex}.wayPoints, 2)
                self.AldyBaggerBotUR3 = self.AldyBaggerBotUR3.addTraj(self.ConveyorBelt.items{itemIndex}.wayPoints{i});
            end
        end
        
    end
end