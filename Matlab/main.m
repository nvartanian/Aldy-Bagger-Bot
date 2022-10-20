%% Robot Controll, structure only so far...
clear All
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
gripper = "gripperClass";
robot = AldyBaggerBot(UR3, gripper);
env = AldyStore(robot);

%run
state = 1;
timestep = 0;
%profile on
while true
    tic  
    timestep
    switch state
        case 0
            %idle, no errors, waiting for start button press, return to home
            env.idle = true;
        case 1
            %packing bags
            env.idle = false;
        case 2
            %both bags full, waiting for bags to be reset and start button press
            env.idle = true;
        case 3
            %stopped, errors (such as eStop)
        case 4
            %safe / jog

        otherwise
            %error, stop
    end
    env = env.stepStore();
    drawnow();
    timestep = timestep+1;
    pause(0.04 - toc) %25hz
end

%profile viewer