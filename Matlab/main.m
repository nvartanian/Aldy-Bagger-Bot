%% Robot Controll, structure only so far...

%initialise environment
gripper = "gripperClass"....
robot = AldyBaggerBot(UR3, gripper);
env = AldyStore(robot);

%run
state = 0;
while(1)
    %100hz
    switch state
        case 0
            %idle, no errors, waiting for start button press, return to home 
        case 1
            %packing bags
            env.stepStore();
        case 2
            %both bags full, waiting for bags to be reset and start button press
        case 3
            %stopped, errors (such as eStop)
        case 4
            %safe / jog

        otherwise
            %error, stop
    end
end