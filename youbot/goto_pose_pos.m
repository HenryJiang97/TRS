function control(displayMap)

% TRS tutorial lab.
% This script computes a path from the robot's initial position to another
% point on the map.
% Search below for a box that looks like this:

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Your task: compute the velocities that take the youBot to the point
  % "target".
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ... and fill it in to make the robot follow its path.

% (C) Copyright Renaud Detry 2013.
% Distributed under the GNU General Public License.
% (See http://www.gnu.org/copyleft/gpl.html)
close all
global displayMap;
global tr;

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
fprintf('Connection %d to remote API server open.\n', id);

% Make sure we close the connexion whenever the script is interrupted.
cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));

% This will only works in "continuous remote API server service"
% See http://www.v-rep.eu/helpFiles/en/remoteApiServerSide.htm
res = vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);
% We're not checking the error code - if vrep is not run in continuous remote
% mode, simxStartSimulation could return an error.
% vrchk(vrep, res);

% Retrieve all handles, and stream arm and wheel joints, the robot's pose,
% the Hokuyo, and the arm tip pose.
h = youbot_init(vrep, id);

timestep = .05;
wheelradius = 0.0937/2; % This value may be inaccurate. Check before using.

% Parameters for controlling the youBot's wheels:
forwBackVel = 0;
leftRightVel = 0;
rotVel = 0;

disp('Starting robot');

% One map cell is 25cm large
global cellsize;
cellsize = .25;

if displayMap,
  mapFig = figure(1);
  axis equal;
  axis([-7.8 7.8 -7.8 7.8]);
  drawnow;
end

armJointRanges = [-2.9496064186096,2.9496064186096;
				 -1.5707963705063,1.308996796608;
				 -2.2863812446594,2.2863812446594;
				 -1.7802357673645,1.7802357673645;
				 -1.5707963705063,1.5707963705063 ];

startingJoints = [0,30.91*pi/180,52.42*pi/180,72.68*pi/180,0];

% In this demo, we move the arm to a preset pose:
pickupJoints = [90*pi/180, 19.6*pi/180, 113*pi/180, -41*pi/180, 0*pi/180];

res = vrep.simxPauseCommunication(id, true); vrchk(vrep, res);
for i = 1:5,
res = vrep.simxSetJointTargetPosition(id, h.armJoints(i),...
									 startingJoints(i),...
									 vrep.simx_opmode_oneshot);
vrchk(vrep, res, true);
end
res = vrep.simxPauseCommunication(id, false); vrchk(vrep, res);

% House map. Has 0 for free space and 1 for obstacles.
load('map')
% House map, with all obstacles dilated by one extra cell.
load('fmap')

[res youbotPos] = vrep.simxGetObjectPosition(h.id, h.ref, -1,vrep.simx_opmode_buffer); vrchk(vrep, res);
[res youbotEuler] = vrep.simxGetObjectOrientation(h.id, h.ref,...
                                 -1, vrep.simx_opmode_buffer); vrchk(vrep, res);
[res dog] = vrep.simxGetObjectPosition(h.id, h.Dog, -1,vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res plant] = vrep.simxGetObjectPosition(h.id, h.Plant, -1,vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res pumpkin] = vrep.simxGetObjectPosition(h.id, h.Pumpkin, -1,vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res Trash] = vrep.simxGetObjectPosition(h.id, h.Trash, -1,vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res Tricycle] = vrep.simxGetObjectPosition(h.id, h.Tricycle, -1,vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);



[res blueCylinder] = vrep.simxGetObjectPosition(h.id, h.blueCylinder, -1,vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res yellowRectangle] = vrep.simxGetObjectPosition(h.id, h.yellowRectangle, -1,vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);

[res greenRectangle] = vrep.simxGetObjectPosition(h.id, h.greenRectangle, -1,vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);
[res greenRectangle_O] = vrep.simxGetObjectOrientation(h.id, h.greenRectangle, -1,vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
%yellowRectangle

[res homeGripperPosition] = vrep.simxGetObjectPosition(id, h.ptip,h.armRef,vrep.simx_opmode_buffer);
   vrchk(vrep, res, true);
   
pause(.3);


res = vrep.simxSynchronous(h.id, true); vrchk(vrep, res);
t = 0;
flagx=0;
flagz=0;
  fsm='rotate';
%   [res greenRectangle] = vrep.simxGetObjectPosition(id, h.greenRectangle, h.armRef,...
%                                            vrep.simx_opmode_oneshot_wait)
%    [res greenRectangle_O] = vrep.simxGetObjectOrientation(id, h.greenRectangle, -1,...
%                                            vrep.simx_opmode_oneshot_wait)
% T_ow=se2(youbotPos(1),-3.83,youbotEuler(3))
% P_cw=[greenRectangle(1) greenRectangle(2) greenRectangle_O(3)]'
% P_oc=inv(T_ow)*P_cw
while true,
  [res youbotPos] = vrep.simxGetObjectPosition(h.id, h.ref, -1,...
                                     vrep.simx_opmode_buffer); vrchk(vrep, res);
  [res youbotEuler] = vrep.simxGetObjectOrientation(h.id, h.ref, -1,...
                                     vrep.simx_opmode_buffer); vrchk(vrep, res);
  
  target = [-3.738; -6.163;1.2217-1.5708];%
  
  forwBackVel = 0;
  leftRightVel = 0;
  rotVel = 0;

    if strcmp(fsm, 'rotate'),
		P_ow=[target(1);target(2);1];
		T_cw=se2(youbotPos(1),youbotPos(2),youbotEuler(3));
		P_oc=inv(T_cw)*P_ow;  %get the object position relative to the youbot_center coordinate
		P_oc;
		if P_oc(2)>=0
			if P_oc(1)>0 
				angle=atan(P_oc(2)/P_oc(1));
				angl=angle-pi/2;   
			end
			if P_oc(1)<0 
				angle=atan(P_oc(2)/P_oc(1));
				angl=angle+pi/2;
			end
		end
		if P_oc(2)<0
			if P_oc(1)>0 
				angle=atan(P_oc(2)/P_oc(1));
				angl=angle+pi/2;   
			end
			if P_oc(1)<0 
				angle=atan(P_oc(2)/P_oc(1));
				angl=angle-pi/2;
			end
		end
		if P_oc(1)==0
			angl=0;
		end
		angl;
		youbotEuler(3);
	    rotVel =30*angl-10*(-youbotEuler(3)+target(3)); %-10*(target(3)+youbotEuler(3))
        %rotVel=-1*rotVel;
        fsm = 'drive';
		rotVel;
		%if sqrt((youbotPos(1)-target(1))^2+(youbotPos(2)-target(2))^2)<0.03 && abs(angdiff(youbotEuler(3),target(3)))<0.02
		%	fsm = 'finished';
		%end
    elseif strcmp(fsm, 'drive'),
        forwBackVel =sign(P_oc(2))* 200*sqrt((youbotPos(1)-target(1))^2+(youbotPos(2)-target(2))^2);
        %forwBackVel=-1*forwBackVel;
        if abs(forwBackVel) < .001,
            forwBackVel = 0;
        end
         fsm = 'rotate';
		if sqrt((youbotPos(1)-target(1))^2+(youbotPos(2)-target(2))^2)<0.03 && abs(angdiff(youbotEuler(3),target(3)))<0.02
			fsm = 'prepare';
		end
		%disp(fsm);
	elseif strcmp(fsm, 'prepare'),
		%res = vrep.simxSetIntegerSignal(id, 'km_mode', 0,vrep.simx_opmode_oneshot_wait);
         for i = 1:5,
             res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), pickupJoints(i), vrep.simx_opmode_oneshot);
             vrchk(vrep, res, true);
        end
		%pause();
		%res = vrep.simxSetIntegerSignal(id, 'km_mode', 2,vrep.simx_opmode_oneshot_wait);
		fsm = 'extend';
		disp(fsm);
       % break;
    elseif strcmp(fsm, 'extend'),
        [res tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef,...
                                           vrep. simx_opmode_buffer);
        vrchk(vrep, res, true);
        if norm(tpos-[0.3259 -0.0010 0.2951]) < .002,
             res = vrep.simxSetIntegerSignal(id, 'km_mode', 2,...
                                   vrep.simx_opmode_oneshot_wait);
             fsm = 'reachout';
        end
       
       
 	elseif strcmp(fsm, 'reachout'),
%         [res tpos_w] = vrep.simxGetObjectPosition(id, h.ptip, -1,vrep. simx_opmode_oneshot_wait)

        
        [res tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef,...
                                           vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);   
        if tpos(1) <.43,
            tpos(1) = tpos(1)+.01;
        else
            flagx=1;
            
        end
        if tpos(3) >.21,
            tpos(3) = tpos(3)-.01;
        else
            flagz=1;
        end
        
        res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, tpos,...
                                    vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
        if flagx==1 && flagz==1
            fsm='grasp';
        end
 		%disp(fsm);
 	elseif strcmp(fsm, 'grasp'),
 		res = vrep.simxSetIntegerSignal(id, 'gripper_open', 0,...
                                    vrep.simx_opmode_oneshot_wait);
         vrchk(vrep, res);
         pause(4);
         res = vrep.simxSetIntegerSignal(id, 'km_mode', 0,...
                                    vrep.simx_opmode_oneshot_wait);
         fsm = 'backoff';
 		%disp(fsm);
	elseif strcmp(fsm, 'backoff'),
		for i = 1:5,
            res = vrep.simxSetJointTargetPosition(id, h.armJoints(i),...
                                         startingJoints(i),...
                                         vrep.simx_opmode_oneshot);
            vrchk(vrep, res, true);
        end
        [res tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef,...
                                           vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        if norm(tpos-homeGripperPosition) < .02,
            res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1,...
                                   vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
        end
        if norm(tpos-homeGripperPosition) < .002,
            fsm = 'finished';
        end
	elseif strcmp(fsm, 'finished'),
 		break;
	
    end
  % Tips:
  % homtrans(T, VECTOR)  transforms VECTOR by T.
  % inv(T)  inverts T
  % atan2 and angdiff can be useful if you want to control the robot's
  %                   orientation.
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  % Update wheel velocities
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

  if displayMap,
    [X,Y] = meshgrid((-7.5+cellsize/2):cellsize:(7.5-cellsize/2),...
                     (-7.5+cellsize/2):cellsize:(7.5-cellsize/2));
    plot(X(map==1), Y(map==1), '*r', youbotPos(1), youbotPos(2), 'ob',...
         7.5, 0, 'or', 0, 7.5, 'og',...
         target(1), target(2), 'b');
    axis equal;
    axis([-7.8 7.8 -7.8 7.8]);
  end
  drawnow;
  vrep.simxSynchronousTrigger(id);
  t = t + timestep;
end

end % main function


function i = ij(x)
  global cellsize;
  n = 15/cellsize;
  i = max(min(floor(x/cellsize)+n/2+1, n), 1);
end

function x = xy(i)
  global cellsize;
  n = 15/cellsize;
  x = (i-n/2-1)*cellsize+cellsize/2;
end

