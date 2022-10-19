classdef AldyBaggerBot
    properties
        robot;
        gripper;
        
        homePose = [0,0,0,0,0,0];
        
        path = [];
        
        %status
        eStop = false;
    end
    methods
        %constructor
        function obj = AldyBaggerBot(robot, gripper)
            obj.robot = robot;
            obj.gripper = gripper;
        end
        
        %methods
        function self = stepRobot(self)
            if self.eStop == false
                if size(self.path, 2) > 0
                    self.robot.model.animate(self.path(1)); %move to next pose in path
                    if size(self.path, 2) > 2
                        self.path = self.path(2:end); %remove from path
                    else
                    %last pose in path
                        self.path = [];
                    end
                end
            end
        end
        
        function traj = calculateTraj(self, startTransform, endTransform)
            traj = [0,0,0,0,0,0]; %TODO
            %check collisions
        end
        
    end
end