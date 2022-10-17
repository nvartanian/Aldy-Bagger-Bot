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
        
        %status
        eStop = false;
        
    end
    methods
        %constructor
        function obj = AldyStore(AldyBaggerBot)
            obj.AldyBaggerBot = AldyBaggerBot;
            beltT = obj.transform * transl(0, 1, 1);
            trplot(beltT);
            obj.ConveyorBelt = ConveyorBelt(beltT, 2, 0.4); %create ConveyorBelt obj
            %obj.BaggingArea = BaggingArea(); %create BaggingArea obj
            
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
            %self.AldyBaggerBot.stepRobot();
        end
        
    end
end