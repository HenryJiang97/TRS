function house_control(displayMap)
%%Having connection


%Map
global displayMap;

if nargin < 1,
  displayMap = true
end


disp('Program started');
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);

if id < 0,
  disp('Failed connecting to remote API server. Exiting.');
  vrep.delete();
  return;
end

cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));

res = vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Initialize parameters

h = youbot_init(vrep, id);

timestep = .05;
wheelRadius = 0.0937/2;

forwBackVel = 0;
leftRightVel = 0
rotVel = 0;

global cellsize;
cellsize = .25;

%Map
if displayMap,
  mapFig = figure(1);
  axis equal;
  axis([-7.8 7.8 -7.8 7.8]);
  drawnow;
end


load('map')

load('fmap')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Design a path

[res youbotPos] = vrep.simxGetObjectPosition(h.id, h.ref, -1,...
                                     vrep.simx_opmode_buffer); vrchk(vrep, res);
[res youbotEuler] = vrep.simxGetObjectOrientation(h.id, h.ref, -1,...
                                     vrep.simx_opmode_buffer); vrchk(vrep, res);


pause(.3);

goal = [ ij(-5.3), ij(1.5) ];
dx = DXform(double(fmap), 'private');
dx.plan([goal(2); goal(1)]);

%Map
figure(2), dx.plot(); figure(1);

traj = dx.path([ij(youbotPos(1)) ; ij(youbotPos(2))])';

traj = [ xy(traj(1,:)); xy(traj(2,:)) ];

save('traj', 'traj');


res = vrep.simxSynchronous(h.id, true); vrchk(vrep, res);
t = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Main code(Calculation)

fsm = 'rotate_home';

while true,
    [res youbotPos] = vrep.simxGetObjectPosition(h.id, h.ref, -1,...
                                     vrep.simx_opmode_buffer); vrchk(vrep, res);
    [res youbotEuler] = vrep.simxGetObjectOrientation(h.id, h.ref,...
                                 -1, vrep.simx_opmode_buffer); vrchk(vrep, res);

    target = traj(:,1);

    if size(traj, 2) > 1 & sqrt((youbotPos(1) - traj(1,1))^2 + (youbotPos(2) - traj(2,1))^2) < 0.5
        traj = traj(:,2:end);
    end

    if strcmp(fsm, 'rotate_home'),

        target = traj(:,1);

        if size(traj, 2) > 1 & sqrt((youbotPos(1) - traj(1,1))^2 + (youbotPos(2) - traj(2,1))^2) < 0.5
            traj = traj(:,2:end);
        end

        P_ow = [target(1);target(2);1];
		T_cw = se2(youbotPos(1),youbotPos(2),youbotEuler(3));
		P_oc = inv(T_cw) * P_ow;  %get the object position relative to the youbot_center coordinate
		if P_oc(1) > 0
			angle = atan(P_oc(2) / P_oc(1));
			angl = angle - pi/2;
		end
		if P_oc(1) < 0
			angle = atan(P_oc(2) / P_oc(1));
			angl = angle + pi / 2;
		end
		if P_oc(1) == 0
			angl = 0;
        end

	    rotVel =10 * angl;

        fsm = 'drive_home';

    elseif strcmp(fsm, 'drive_home'),

        forwBackVel = 10 * sqrt((youbotPos(1) - target(1))^2 + (youbotPos(2) - target(2))^2)

        fsm = 'rotate_home';

        if sqrt((youbotPos(1) - target(1))^2 + (youbotPos(2) - target(2))^2) < 0.01
            fsm='rotate_home2';
            i = 1;
            forwBackVel = 0;
            rotVel = 0;

            pause(.5);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            [res youbotPos] = vrep.simxGetObjectPosition(h.id, h.ref, -1,...
                                     vrep.simx_opmode_buffer); vrchk(vrep, res);
            [res youbotEuler] = vrep.simxGetObjectOrientation(h.id, h.ref, -1,...
                                     vrep.simx_opmode_buffer); vrchk(vrep, res);

            pause(.3);

            goal = [ ij(-5.25), ij(-1.175) ];
            dx = DXform(double(map), 'private');
            dx.plan([goal(2); goal(1)]);

            traj = dx.path([ij(youbotPos(1)) ; ij(youbotPos(2))])';

            traj = [ xy(traj(1,:)); xy(traj(2,:)) ];

            save('traj', 'traj');

        end
        
    elseif strcmp(fsm, 'rotate_home2'),

        target = traj(:,1);

        if size(traj, 2) > 1 & sqrt((youbotPos(1) - traj(1,1))^2 + (youbotPos(2) - traj(2,1))^2) < 0.5
            traj = traj(:,2:end);
        end

        P_ow = [target(1);target(2);1];
		T_cw = se2(youbotPos(1),youbotPos(2),youbotEuler(3));
		P_oc = inv(T_cw) * P_ow;  %get the object position relative to the youbot_center coordinate
		if P_oc(1) > 0
			angle = atan(P_oc(2) / P_oc(1));
			angl = angle - pi/2;
		end
		if P_oc(1) < 0
			angle = atan(P_oc(2) / P_oc(1));
			angl = angle + pi / 2;
		end
		if P_oc(1) == 0
			angl = 0;
        end

	    rotVel =10 * angl;

        fsm = 'drive_home2';

    elseif strcmp(fsm, 'drive_home2'),

        forwBackVel = 10 * sqrt((youbotPos(1) - target(1))^2 + (youbotPos(2) - target(2))^2)

        fsm = 'rotate_home2';

        if sqrt((youbotPos(1) - target(1))^2 + (youbotPos(2) - target(2))^2) < 0.01
            fsm='finished';
            i = 1;
            forwBackVel = 0;
            rotVel = 0;

            pause(.5);

        end

        
        elseif strcmp(fsm, 'finished'),
            break;
            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Send command to robot

    res = vrep.simxPauseCommunication(h.id, true); vrchk(vrep, res);

    vrep.simxSetJointTargetVelocity(h.id, h.wheelJoints(1),...
                                  -forwBackVel-leftRightVel+rotVel,...
                                  vrep.simx_opmode_oneshot); vrchk(vrep, res);
    vrep.simxSetJointTargetVelocity(h.id, h.wheelJoints(2),...
                                  -forwBackVel+leftRightVel+rotVel,...
                                  vrep.simx_opmode_oneshot); vrchk(vrep, res);
    vrep.simxSetJointTargetVelocity(h.id, h.wheelJoints(3),...
                                  -forwBackVel-leftRightVel-rotVel,...
                                  vrep.simx_opmode_oneshot); vrchk(vrep, res);
    vrep.simxSetJointTargetVelocity(h.id, h.wheelJoints(4),...
                                  -forwBackVel+leftRightVel-rotVel,...
                                  vrep.simx_opmode_oneshot); vrchk(vrep, res);
    res = vrep.simxPauseCommunication(h.id, false); vrchk(vrep, res);


    %Map
    if displayMap,
        [X,Y] = meshgrid((-7.5+cellsize/2):cellsize:(7.5-cellsize/2),...
                     (-7.5+cellsize/2):cellsize:(7.5-cellsize/2));
        plot(X(map==1), Y(map==1), '*r', youbotPos(1), youbotPos(2), 'ob',...
                     7.5, 0, 'or', 0, 7.5, 'og',...
                     traj(1,:), traj(2,:), 'b');
    axis equal;
    axis([-7.8 7.8 -7.8 7.8]);
    end
    drawnow;


    vrep.simxSynchronousTrigger(id);
    t = t+timestep;
end

end

end

%%Functions to deal with ij and xy coordinate


function i = ij(x)
    global cellsize;
    n = 15 / cellsize;
    i = max(min(floor(x / cellsize) + n / 2 + 1, n), 1);
end

function x = xy(i)
    global cellsize;
    n = 15 / cellsize;
    x = (i - n / 2 - 1) * cellsize + cellsize / 2;
end
