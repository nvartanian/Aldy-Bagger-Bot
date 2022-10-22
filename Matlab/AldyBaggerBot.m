classdef AldyBaggerBot
    properties
        robot;
        gripper;
        
        homePose = [0,-pi/2,0,0,0,0];
        poseGuess = deg2rad([0,-60,120,-150,-90,0]);
        itemGuess =[3*pi/4,45,90,-45,90,0];
        
        path = [];
        
        %status
        eStop = false;
    end
    methods
        %constructor
        function obj = AldyBaggerBot(robot, gripper)
            obj.robot = robot;
            obj.gripper = gripper;
            
            %% adjust robot qlim's
            %robot.model.qlim(2,1) = pi/4;
            %robot.model.qlim(2,2) = -5*pi/4;
        end
        
        %methods
        function self = stepRobot(self)
            if self.eStop == false
                %gripper movement
                
                
                %robot movement
                if size(self.path, 1) > 0
                    self.robot.model.animate(self.path(1,:)); %move to next pose in path
                    if size(self.path, 1) > 2
                        self.path = self.path(2:end,:); %remove from path
                    else
                        self.path = [];%last pose in path
                    end
                end
            end
        end
        
        
        function r = inRange(self, transform)
            r = false;
            qtarget = self.robot.model.ikcon(transform,self.poseGuess);
            if max(abs(self.robot.model.fkine(qtarget) - transform)) > 0.005
                return
            end
            r = true;
            %TODO: determine if in range, return true if in range of robot
        end
        
        function self = addTraj(self, endTransform)
            steps = 50;
            try qStart = self.path(end,:); %last pose in path
            catch
                qStart = self.homePose;
            end
            qEnd = self.robot.model.ikcon(endTransform, self.poseGuess);
            self.path = [self.path; jtraj(qStart, qEnd, steps)]; %add to path
        end
        
    end
end