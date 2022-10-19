classdef FetchRobot < handle
    properties
        %> Robot model
        model;
        
        %>Workspace 
        
        workspace;
        scale = 0.2; 
        
        %> Flag to indicate if gripper is used
        useGripper = false;     

        % For calculations
        maxReachFetchRobotX;
        maxReachFetchRobotY;
        maxReachFetchRobotZ;
        volumeFetchRobot;
        qlim; 
        endEffectorPoses;
        q;
        base;


        armTraj;
        taskcompleted = false;

        % State machine
        previousState = 0; 
        currentState = 0; 
        targetedBrickID = 0;
        steps = 0; 
        %qHome = [0, 0, 0, 0, 0, 0, 0, 0];
        qHome = [0 1.32 0.7 0.0000290135 -2 -0.000123035 -0.57 0.00123035]
        % qHome =zeros(1,8);
       

    end
    
    methods%% Class for FetchRobot robot simulation
function self = FetchRobotGripper(useGripper)
    if nargin < 1
        useGripper = false;
    end
    self.useGripper = useGripper;
    
%> Define the boundaries of the workspace
        
% robot = 
self.GetFetchRobotRobot();

% robot = 
self.PlotAndColourRobot();%robot,workspace);

end
%% Get and plot Robot
function self = FetchRobot(baseFetchRobot, workspace)
    
    % Set up FetchRobot robot workspace        
    %> robot = 
    self.workspace = workspace; 
    self.GetFetchRobotRobot();

    %> Define the base location
    self.model.base = baseFetchRobot;
    self.base = baseFetchRobot;

    %> robot = 
    self.PlotLinksFetchRobot(); % only for stick model
end

%% GetFetchRobotRobot
% Given a name (optional), create and return a UR3 robot model
function GetFetchRobotRobot(self)
    pause(0.001);
    name = ['FetchRobot_',datestr(now,'yyyymmddTHHMMSSFFF')];

    % https://www.google.com/url?sa=i&url=http%3A%2F%2Fragazzini.me.vt.edu%2Fpdf%2FLearning_IK_ID_GAN_RAS_2020.pdf&psig=AOvVaw1Rba0hDu8iqFIiWcYX9SmQ&ust=1650853547949000&source=images&cd=vfe&ved=0CA0QjhxqFwoTCPC9kO7Sq_cCFQAAAAAdAAAAABAm

    L0 = 0.110;  % x
    D1 = 0.0;  % 
    L1 = 0.1197;  % 
    
    L2 = 0.1163;  % Shoulder pan joint
    L3 = 0.17181;  % Shoulder lift joint
    L4 = 0.18093;  % Upper arm roll joint
    L5 = 0.1658;  % elbow flex joint
    L6 = 0.15675;  % forarm roll joint
    L7 = 0.1166;  % wrist flex joint
    L8 = 0.04224;  % wrist roll joint
    L9 = 0.12193;  % gripper joint

    L(1) = Link([pi     L1+D1   L0         0      1]); % PRISMATIC Link shulder pan
    L(2) = Link([0      L3      L2        -pi/2   0]); % SHoulder lift
    L(3) = Link([0      0       0         -pi/2   0]); % upper arm rol joint
    L(4) = Link([0      L4+L5   0          pi/2   0]);
    L(5) = Link([0      0       0         -pi/2   0]);
    L(6) = Link([0      L6+L7   0          pi/2	  0]);
    L(7) = Link([0      0       0         -pi/2   0]);
    L(8) = Link([0      L8+L9   0          0      0]);
    %            ?      d       x          Alpa   type
    
    % Incorporate joint limits
    L(1).qlim = [0 0.400];
    L(2).qlim = [-23/45*pi, 23/45*pi];
    L(3).qlim = [-7/18*pi, 29/60*pi];
    L(4).qlim = [-360 360]*pi/180; %continuous not sure how to do that
    L(5).qlim = [-43/60*pi, 43/60*pi];
    L(6).qlim = [-360 360]*pi/180; %continuous
    L(7).qlim = [-25/36*pi, 25/36*pi ];
    L(8).qlim = [-360 360]*pi/180; %continuous

    L(3).offset = -pi/2;
    %L(5).offset = -pi/2; % commented out due to gazebo values being to
    %differnt or taking into account offset. 

    
    %fetchBot = SerialLink([L(1) L(2) L(3) L(4) L(5) L(6) L(7) L(8)]);
    %fetchBot.base = transl(0,0,0);
                                    
    %workspace = [-1.5 1.5 -1.5 1.5 -1.5 1.5]; 
    %scale = 0.2;        
    %q = zeros(1,8);                                                           
    %fetchBot.plot(q,'workspace',workspace,'scale',scale);                  
    %hold on 

    % FetchRobot DH parameters found at: 
    % https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
    
    self.model = SerialLink([L(1) L(2) L(3) L(4) L(5) L(6) L(7) L(8)],'name','FetchRobot');
    self.model.base = self.base; 
end

%% teaching
function teaching(self)
    self.model.teach();
end

%% 
function PlotLinksFetchRobot(self)
    self.model.plot(self.qHome,'workspace',self.workspace,'scale',self.scale, 'noarrow'); 
    hold on
end
%% PlotAndColourRobot
% Given a robot index, add the glyphs (vertices and faces) and
% colour them in if data is available 
function PlotAndColourRobot(self)%robot,workspace)
    for linkIndex = 0:self.model.n
        if self.useGripper && linkIndex == self.model.n
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['FetchRobotLink',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
        else
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['FetchRobotLink',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
        end
        self.model.faces{linkIndex+1} = faceData;
        self.model.points{linkIndex+1} = vertexData;
    end

    % Display robot
    self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
    if isempty(findobj(get(gca,'Children'),'Type','Light'))
        camlight
    end  
    self.model.delay = 0;

    % Try to correctly colour the arm (if colours are in ply file data)
    for linkIndex = 0:self.model.n
        handles = findobj('Tag', self.model.name);
        h = get(handles,'UserData');
        try 
            h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                          , plyData{linkIndex+1}.vertex.green ...
                                                          , plyData{linkIndex+1}.vertex.blue]/255;
            h.link(linkIndex+1).Children.FaceColor = 'interp';
        catch ME_1
            disp(ME_1);
            continue;
        end
    end
end 



%% Calculate the arms max reach and volume

% Reach
% Calculat max reach via teach (qMax). 
% Animate moving to that position

function CalculateMaxReach(self, baseFetchRobot)

disp('Please press Enter to continue');
pause(); 
disp('FetchRobot Max Reach X axis');
    for i =0:-pi/10:-pi/2
        self.model.plot([pi, 0, 0, i, 0, 0]);
    end
    self.q =[pi, 0, 0, -pi/2, 0, 0];
    maxReachKineX = self.model.fkine(self.q);
    maxReachFetchRobotX = ([abs(maxReachKineX(1,4) - baseFetchRobot(1,4)); 
                     abs(maxReachKineX(2,4) - baseFetchRobot(2,4)); 
                     abs(maxReachKineX(3,4) - baseFetchRobot(3,4))]);

    self.maxReachFetchRobotX = maxReachFetchRobotX;

    disp('Please press Enter to continue');
    pause(); 
    disp('FetchRobot Max Reach Y axis');
    
    for i =pi:pi/10:(3*pi/2)
        self.model.plot([i, 0, 0, -pi/2, 0, 0]);
    end
    self.q =[(3*pi/2), 0, 0, -pi/2, 0, 0];
    maxReachKineY = self.model.fkine(self.q);
    maxReachFetchRobotY = ([abs(maxReachKineY(1,4) - baseFetchRobot(1,4)); 
                     abs(maxReachKineY(2,4) - baseFetchRobot(2,4)); 
                     abs(maxReachKineY(3,4) - baseFetchRobot(3,4))]);
    
    self.maxReachFetchRobotY = maxReachFetchRobotY;

    disp('Please press Enter to continue');
    pause(); 
    disp('FetchRobot Max Reach Z axis');

    for i =0:-pi/10:-pi/2
        self.model.plot([(3*pi/2), i, 0, -pi/2, 0, 0]);
    end
    self.q =[(3*pi/2), -pi/2, 0, -pi/2, 0, 0];
    maxReachKineZ = self.model.fkine(self.q);
    maxReachFetchRobotZ = ([abs(maxReachKineZ(1,4) - baseFetchRobot(1,4)); 
                     abs(maxReachKineZ(2,4) - baseFetchRobot(2,4)); 
                     abs(maxReachKineZ(3,4) - baseFetchRobot(3,4))]);

    self.maxReachFetchRobotZ = maxReachFetchRobotZ;

    disp('Please press Enter to continue');
    pause(); 

    self.model.plot(self.qHome);

    disp ('Max Arm Reach from FetchRobot Base (x,y,z) = ')
    display(self.maxReachFetchRobotX(1)), display(self.maxReachFetchRobotY(2)), display(self.maxReachFetchRobotZ(3))
end

%%
function FetchRobotReachVolume(self)
    % Cycle through models Joint limits
    self.qlim = self.model.qlim();
    counter = 1;
    for q1 = self.qlim(1,1):(pi/10):self.qlim(1,2)
        for q2 = self.qlim(2,1):(pi/10):self.qlim(2,2)
            for q3 = self.qlim(3,1):(pi/10):self.qlim(3,2)
                for q4 = self.qlim(4,1):(pi/10):self.qlim(4,2)
                    endEff = self.model.fkine([q1, q2, q3, q4, 0, 0, 0, 0]);
                    self.endEffectorPoses(counter,:) = endEff(1:3,4)';
                    counter = counter + 1;
                end
            end
        end
    end
    % Plot poses of end effectors
    % https://au.mathworks.com/help/matlab/ref/plot3.html
    volume = plot3(self.endEffectorPoses(:,1), ...
                self.endEffectorPoses(:,2), ...
                self.endEffectorPoses(:,3));
                
    % https://au.mathworks.com/help/matlab/ref/convhull.html#mw_f9cda693-9c7a-40a0-84e0-7cee7b2af88b
    [~, self.volumeFetchRobot] = convhull(self.endEffectorPoses(:,1), ...
                                   self.endEffectorPoses(:,2), ...
                                   self.endEffectorPoses(:,3));
    disp ('FetchRobot Arm Reach Volume = ')
    display (self.volumeFetchRobot)
    disp('Please press Enter to continue');
    pause(); 
    
    set(volume, 'Visible', 'off');
end
    end
end