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
        
        personVertices;
        
        gripperOffset = transl(0,0,0.22);
        
        scanTransform = eye(4) * transl(0.25,1.8,1.1) * trotx(deg2rad(180)); %TODO, work out pose to pass scanner. Trial/error, plot transfrom and check.
        kukaVSTransform = eye(4) * transl(0,1.2,1.2) * trotz(deg2rad(75));
        
        %status
        idle = true;
        eStop = false;
        visualServoing = false
        
    end
    methods
        %constructor
        function obj = AldyStore(AldyBaggerBot6DOF,AldyBaggerBot7DOF)
            obj.AldyBaggerBotUR3 = AldyBaggerBot6DOF;
            obj.AldyBaggerBotUR3.robot.model.base = obj.transform * transl(0, 1.75, 0.75) * trotz(pi);
            obj.AldyBaggerBotUR3.gripper.moveGripper(obj.AldyBaggerBotUR3.robot.model.fkine(obj.AldyBaggerBotUR3.homePose), [0,0]);
            obj.AldyBaggerBotUR3.robot.model.animate(obj.AldyBaggerBotUR3.homePose);
            obj.AldyBaggerBotUR3.gripper.LeftFinger.animate(deg2rad(-30));
            obj.AldyBaggerBotUR3.gripper.RightFinger.animate(deg2rad(-30));
            
            obj.AldyBaggerBotKUKA = AldyBaggerBot7DOF; % add kuka bot
            obj.AldyBaggerBotKUKA.robot.model.base = obj.transform * transl(-0.3, 0.5, 0.8) * trotz(pi);
            obj.AldyBaggerBotKUKA.gripper.moveGripper(obj.AldyBaggerBotKUKA.robot.model.fkine(obj.AldyBaggerBotKUKA.homePose), [0,0]);
            obj.AldyBaggerBotKUKA.robot.model.animate(obj.AldyBaggerBotKUKA.homePose);
            
            beltT = obj.transform * transl(0.11, 1.5, 0.75);
            obj.ConveyorBelt = ConveyorBelt(beltT, 3, 0.1); %create ConveyorBelt obj
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
            [f,v,data] = plyread('person.ply','tri');
            obj.personVertices = v;
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
            
            % collision detection
            collisionVertices = self.personVertices;
            collisionVertices(:, 1) = collisionVertices(:, 1) + self.Person.body.base(1, 4);
            collisionVertices(:, 2) = collisionVertices(:, 2) + self.Person.body.base(2, 4);
            collisionVertices(:, 3) = collisionVertices(:, 3) + self.Person.body.base(3, 4);
            if checkCollision(self.AldyBaggerBotUR3.robot.model, collisionVertices, 0.1) == true
                self.eStop = true; %trigger E-stop.
                return;
            end

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
                        if size(self.AldyBaggerBotUR3.gripperPath) < 1
                            self.AldyBaggerBotUR3.gripperPath = jtraj([0,0], [deg2rad(-30),deg2rad(-30)], 50);
                        end
                        continue
                    end
                    %not at bag yet, keep brick on robot
                    self.ConveyorBelt.items{i} = self.ConveyorBelt.items{i}.moveItem(self.AldyBaggerBotUR3.robot.model.fkine(self.AldyBaggerBotUR3.robot.model.getpos) * self.gripperOffset * trotx(deg2rad(180)));
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
                        if size(self.AldyBaggerBotUR3.gripperPath) < 1
                            self.AldyBaggerBotUR3.gripperPath = jtraj([deg2rad(-30),deg2rad(-30)], [0,0], 50);
                        end
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

                if self.AldyBaggerBotUR3.inRange(self.ConveyorBelt.items{i}.transform * self.gripperOffset * transl(0,0,0.05) * trotx(deg2rad(180))) ~= true
                    %warning('Item %d is not in range, continuing to next item', i);
                    continue
                end
                if self.ConveyorBelt.items{i}.heavy == true %find a bag spot that can take a heavy item
                    [bagIndex, slotIndex] = self.BaggingArea.nextHeavyBag();
                    if isa(bagIndex, 'bool') ~= true
                        self.BaggingArea.bags{bagIndex}.heavySlotsFull{slotIndex} = true;
                        self = self.generateBaggingPath(i, self.BaggingArea.bags{bagIndex}.heavySlotsTransform{slotIndex}*self.gripperOffset);
                        self.ConveyorBelt.items{i}.trajCalculated = true;
                        return
                    end
                else %find a bag spot that can take a light item
                    [bagIndex, slotIndex] = self.BaggingArea.nextLightBag();
                    if isa(bagIndex, 'bool') ~= true
                        self.BaggingArea.bags{bagIndex}.lightSlotsFull{slotIndex} = true;
                        self = self.generateBaggingPath(i, self.BaggingArea.bags{bagIndex}.lightSlotsTransform{slotIndex}*self.gripperOffset);
                        self.ConveyorBelt.items{i}.trajCalculated = true;
                        return
                    end
                end
                %could not find a bag spot, return error TODO
                 
            end
            
            %% kuka robot function
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
                    %move item to robot
                    %check if it is close to the goal
                    if max(max(abs(self.AldyBaggerBotKUKA.robot.model.fkine(self.AldyBaggerBotKUKA.robot.model.getpos) - self.ConveyorBelt.items{i}.wayPoints{6}))) < 0.005
                        %return item to the conveyer belt
                        self.ConveyorBelt.items{i}.onRobot = false;
                        continue
                    end
                    %not at bag yet, keep item on robot
                    self.ConveyorBelt.items{i} = self.ConveyorBelt.items{i}.moveItem(self.AldyBaggerBotKUKA.robot.model.fkine(self.AldyBaggerBotKUKA.robot.model.getpos) * self.gripperOffset * trotx(deg2rad(180)));
                    continue
                end
                if self.ConveyorBelt.items{i}.trajCalculatedAlign ~= false
                    %align traj calculated but not yet on kuka
                    %check if item is close enough to collect
                    if max(max(abs(self.AldyBaggerBotKUKA.robot.model.fkine(self.AldyBaggerBotKUKA.robot.model.getpos) - self.ConveyorBelt.items{i}.wayPoints{1}))) < 0.005
                        self.ConveyorBelt.items{i}.onRobot = true;
                    end
                    continue
                end

                if self.AldyBaggerBotKUKA.inRange(self.ConveyorBelt.items{i}.transform * self.gripperOffset * transl(0,0,0.05) * trotx(deg2rad(180))) ~= true
                    %warning('Item %d is not in range, continuing to next item', i);
                    continue
                end
                
                if self.ConveyorBelt.items{i} %Find item to align
                    [bagIndex, slotIndex] = self.BaggingArea.nextHeavyBag();
                    if isa(bagIndex, 'bool') ~= true
                        self.BaggingArea.bags{bagIndex}.heavySlotsFull{slotIndex} = true;
                        self = self.generateBaggingPath(i, self.BaggingArea.bags{bagIndex}.heavySlotsTransform{slotIndex}*self.gripperOffset);
                        self.ConveyorBelt.items{i}.trajCalculated = true;
                        return
                    end

                end
                %could not find a bag spot, return error TODO
                 
            end
            %}
        end
        
        function self = generateBaggingPath(self, itemIndex, slotTransform)            
            %determine robot approach item pose
            self.ConveyorBelt.items{itemIndex}.wayPoints{1} = self.ConveyorBelt.items{itemIndex}.transform * self.gripperOffset * transl(0,0,0.05) * trotx(deg2rad(180));
            
            %robot grab item
            self.ConveyorBelt.items{itemIndex}.wayPoints{2} = self.ConveyorBelt.items{itemIndex}.transform * self.gripperOffset * trotx(deg2rad(180));
            
            %robot move to safe pose
            t = self.ConveyorBelt.items{itemIndex}.transform * self.gripperOffset * transl(0,0,0.05) * trotx(deg2rad(180));
            t(1:3, 1:3) = self.scanTransform(1:3, 1:3);
            self.ConveyorBelt.items{itemIndex}.wayPoints{3} = t;
            
            %robot move past scanner
            self.ConveyorBelt.items{itemIndex}.wayPoints{4} = self.scanTransform;
            
            %robot approach bag
            self.ConveyorBelt.items{itemIndex}.wayPoints{5} = slotTransform * transl(0, 0, 0.1) * trotx(deg2rad(180));
            
            %robot place item
            self.ConveyorBelt.items{itemIndex}.wayPoints{6} = slotTransform * trotx(deg2rad(180));
            
            %robot move away from bag
            self.ConveyorBelt.items{itemIndex}.wayPoints{7} = slotTransform * transl(0, 0, 0.1) * trotx(deg2rad(180));
            
            %robot move to home pose
            %self.ConveyorBelt.items{itemIndex}.wayPoints{8} = self.AldyBaggerBot.robot.model.fkine(AldyBaggerBot.homePose);
            for i = 1:size(self.ConveyorBelt.items{itemIndex}.wayPoints, 2)
                self.AldyBaggerBotUR3 = self.AldyBaggerBotUR3.addTraj(self.ConveyorBelt.items{itemIndex}.wayPoints{i});
            end
        end
        
        function self = generateAlignPath(self, AldyBaggerBotKUKA, itemIndex)            
            %determine robot approach bag pose
            self.BaggingArea.Bag{fullBagIndex}.wayPoints2{1} = self.BaggingArea.Bag{fullBagIndex}.transform * transl(0,0.2,0) * trotx(deg2rad(-90));
            
            %robot move to customer hole
            self.BaggingArea.Bag{fullBagIndex}.wayPoints2{2} = self.BaggingArea.Bag{fullBagIndex}.transform * transl(0.1,0.5,0) * trotx(deg2rad(-90)) * trotz(deg2rad(90)); 
            
            %robot move to customer
            self.BaggingArea.Bag{fullBagIndex}.wayPoints2{3} = self.BaggingArea.Bag{fullBagIndex}.transform * transl(0.5,0.5,0) * trotx(deg2rad(-90)) * trotz(deg2rad(90));
            
            %robot move to customer hole
            self.BaggingArea.Bag{fullBagIndex}.wayPoints2{4} = self.BaggingArea.Bag{fullBagIndex}.transform * transl(0.1,0.5,0) * trotx(deg2rad(-90)) * trotz(deg2rad(90));
            
            %robot move to home pose
            self.BaggingArea.Bag{fullBagIndex}.wayPoints2{5} = self.AldyBaggerBotKUKA.robot.model.fkine(AldyBaggerBotKUKA.homePose);
            
            for i = 1:size(self.BaggingArea.Bag{fullBagIndex}.wayPoints2, 2)
                self.AldyBaggerBotKUKA = self.AldyBaggerBotKUKA.addTraj(self.BaggingArea.Bag{fullBagIndex}.wayPoints2{i});
            end
        end
        
    end
end