classdef AldyBaggerBot
    properties
        robot;
        gripper;
        
        %status
        moving = false;
        eStop = false;
    end
    methods
        %constructor
        function obj = AldyBaggerBot(robot, gripper)
            obj.robot = robot;
            obj.gripper = gripper;
        end
        
        function bagItem(robot, item, bag)
            %robot approach item
            %determine next pose
            
            %check collisions
            
            %move
            while obj.moving
                %check collisions (<5cm?)
                %check errors (e-stop etc)
            end
            %robot grab item
            
            %robot move to safe pose
            
            %robot move past scanner
            
            %robot approach bag
            
            %robot place item
            
            %robot move to safe pose
            
        end
    end
end