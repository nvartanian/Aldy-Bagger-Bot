%% Robot Controll, structure only so far...

%initialise environment
robot = AldyBaggerBot(UR3);

%run
state = 0;
switch state
    case 0
        %idle, no errors, waiting for start button press 
    case 1
        %packing bags
    case 2
        %bag/s full, waiting for bags to be reset and start button press
    case 3
        %stopped, errors
    
    otherwise
        %error, stop
end