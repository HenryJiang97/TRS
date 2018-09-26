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
[res blueCylinder_O] = vrep.simxGetObjectOrientation(h.id, h.blueCylinder, -1,vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res yellowRectangle] = vrep.simxGetObjectPosition(h.id, h.yellowRectangle, -1,vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res redRectangle] = vrep.simxGetObjectPosition(h.id, h.redRectangle, -1,vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res redRectangle_O] = vrep.simxGetObjectOrientation(h.id, h.redRectangle, -1,vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res greenRectangle] = vrep.simxGetObjectPosition(h.id, h.greenRectangle, -1,vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res greenRectangle_O] = vrep.simxGetObjectOrientation(h.id, h.greenRectangle, -1,vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
%yellowRectangle
P_oc=[-0.167;0.42;1];
T_ow=se2(greenRectangle(1),greenRectangle(2),greenRectangle_O(3));
greenRectangle_O(3)
P_cw=T_ow*P_oc  %get the object position relative to the youbot_center coordinate
		

pause(.3);

% Plan a path from the youBot's current position to map cell 30, 20.
goal = [ 30, 20 ];
dx = DXform(double(fmap), 'private');
dx.plan([goal(2); goal(1)]);
figure(2),dx.plot();
 figure(1);
traj = dx.path([ij(youbotPos(1)) ; ij(youbotPos(2))])';

% Convert trajectory to world coordinates:
traj = [ xy(traj(1,:)); xy(traj(2,:)) ];
size(traj);
tr=traj;
% traj = [ x1 x2 x3 ...;
%          y1 y2 y3 ... ];
save('traj', 'traj');
%load('traj'); % In case DXform does not work, load trajectory.

res = vrep.simxSynchronous(h.id, true); vrchk(vrep, res);
t = 0;
  fsm='rotate';
while false,
  [res youbotPos] = vrep.simxGetObjectPosition(h.id, h.ref, -1,...
                                     vrep.simx_opmode_buffer); vrchk(vrep, res);
  [res youbotEuler] = vrep.simxGetObjectOrientation(h.id, h.ref, -1,...
                                     vrep.simx_opmode_buffer); vrchk(vrep, res);
  
  target = traj(:,1);%[-2.625;2.125];%
  
  if size(traj, 2) > 1 & sqrt((youbotPos(1)-traj(1,1))^2+(youbotPos(2)-traj(2,1))^2)<0.5 %mod(int32(1000*t),500) == 0
    traj = traj(:,2:end);
  end
  robotPose = se2(youbotPos(1), youbotPos(2), youbotEuler(3));
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Your task: compute the velocities that take the youBot to the point
  % "target".
  forwBackVel = 0;
  leftRightVel = 0;
  rotVel = 0;

   if strcmp(fsm, 'rotate'),
		P_ow=[target(1);target(2);1];
		T_cw=se2(youbotPos(1),youbotPos(2),youbotEuler(3));
		P_oc=inv(T_cw)*P_ow;  %get the object position relative to the youbot_center coordinate
		if P_oc(1)>0
			angle=atan(P_oc(2)/P_oc(1));
			angl=angle-pi/2;   
		end
		if P_oc(1)<0
			angle=atan(P_oc(2)/P_oc(1));
			angl=angle+pi/2;
		end
		if P_oc(1)==0
			angl=pi/2;
		end
		beta=0.5235;
	   rotVel =10*angl; %angdiff(angl, youbotEuler(3));
       youbotEuler(3);
        fsm = 'drive';
		if abs(angdiff(angl, youbotEuler(3))) < 1/180*pi,
            rotVel = 0;
        end
   elseif strcmp(fsm, 'drive'),
        forwBackVel = 20*((youbotPos(1)-traj(1,1))^2+(youbotPos(2)-traj(2,1))^2);
        if abs(forwBackVel) < .001,
            forwBackVel = 0;
        end
         fsm = 'rotate';
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
         traj(1,:), traj(2,:), 'b');
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
