%% Visual Servoing Demo
close all
clf

%% 
% Create image target 
pStar = [824 200 200 824; 200 200 824 824];

%Create 3D points
P=[1.0,1.0,1.0,1.0;
0.0,0.5,0.5,0.0;
0.75,0.75,0.25,0.25];

r = UR3();             
q0 = [pi/2; -2*pi/3; -pi/3; -pi/6; 0; pi/8];

% Add the camera
cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
'resolution', [1024 1024], 'centre', [512 512],'name', 'UR3camera');


fps = 25;


%gain of the controler
lambda = 0.9;
%depth of the points
depth = mean (P(1,:));

%% Simulation
%Display UR3
Tc0= r.model.fkine(q0);
r.model.animate(q0');
drawnow

% plot camera and points
cam.T = Tc0;

% Display points in 3D and the camera
cam.plot_camera('Tcam',Tc0, 'label','scale',0.15);
ball = plot_sphere(P, 0.05, 'b');
lighting gouraud
light

%Project points to the image
p = cam.plot(P, 'Tcam', Tc0);

%camera view and plotting
cam.clf()
cam.plot(pStar, '*'); % create the camera view
cam.hold(true);
cam.plot(P, 'Tcam', Tc0, 'o'); % create the camera view
pause(2)
cam.hold(true);
cam.plot(P);    % show initial view


%% Loop
timestep = 0;
 while true
        timestep = timestep + 1;
        delete(ball);
        P = P.*[0.995;1;1];
        depth = mean (P(1,:));
        ball = plot_sphere(P, 0.05, 'b');
        % compute the view of the camera
        uv = cam.plot(P);
        
        % compute image plane error as a column
        e = pStar-uv;   % feature error
        e = e(:);
        Zest = [];
        
        % compute the Jacobian
        if isempty(depth)
            % exact depth from simulation (not possible in practice)
            pt = homtrans(inv(Tcam), P);
            J = cam.visjac_p(uv, pt(3,:) );
        elseif ~isempty(Zest)
            J = cam.visjac_p(uv, Zest);
        else
            J = cam.visjac_p(uv, depth );
        end

        % compute the velocity of camera in camera frame
        try
            v = lambda * pinv(J) * e;
        catch
            status = -1;
            return
        end
        %joint velocities
        fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);

        %compute robot's Jacobian and inverse
        J2 = r.model.jacobn(q0);
        Jinv = pinv(J2);
        % get joint velocities
        qp = Jinv*v;

         
         %Maximum angular velocity cannot exceed 180 degrees/s
         ind=find(qp>pi);
         if ~isempty(ind)
             qp(ind)=pi;
         end
         ind=find(qp<-pi);
         if ~isempty(ind)
             qp(ind)=-pi;
         end

        %Update joints 
        q = q0 + (1/fps)*qp;
        r.model.animate(q');

        %Get camera location
        Tc = r.model.fkine(q);
        cam.T = Tc;

        drawnow
        

         pause(1/fps)

        if ~isempty(200) && (timestep > 200)
            break;
        end
        
        %update current joint position
        q0 = q;
 end 


     