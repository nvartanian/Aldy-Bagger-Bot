%% Robot Controll, structure only so far...
clear all
clc
clf
hold on
camlight;

try
   transl(0,0,0); %try a toolbox function to see if it has been run yet
catch exception
   run("rvctools/startup_rvc.m");
end

%initialise environment
gripper1 = Gripper('grip1');
gripper2 = Gripper('grip2');
robotUR3 = AldyBaggerBot6DOF(UR3, gripper1);
robotKUKA = AldyBaggerBot7DOF(KUKA, gripper2);
robotUR3.robot.model.qlim = deg2rad([-160,130;-120,40;-20,160;-190,40;-110,190;-360,360;]);
env = AldyStore(robotUR3,robotKUKA);
gui = AldyGUI(env);

%run
timestep = 0;
% profile on
while true
    tic  
    timestep
    env = gui.updateEnv(env);
    env = env.stepStore();
    gui = gui.updateGUI(env);
    drawnow();
    timestep = timestep+1;
    pause(0.04 - toc) %25hz
end

% profile viewer
