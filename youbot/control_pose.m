function control_pose(displayMap)
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
clear all

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

[res youbotPos] = vrep.simxGetObjectPosition(h.id, h.ref, -1,...
                                     vrep.simx_opmode_buffer); vrchk(vrep, res);
[res youbotEuler] = vrep.simxGetObjectOrientation(h.id, h.ref,...
                                 -1, vrep.simx_opmode_buffer); vrchk(vrep, res);

[res rgbdp]=vrep.simxGetObjectPosition(h.id, h.xyzSensor, h.armRef, vrep.simx_opmode_buffer);  
[res rgbdpo]=vrep.simxGetObjectOrientation(h.id, h.xyzSensor, h.armRef, vrep.simx_opmode_buffer);  
rgbdp
rgbdpo

T=transl(rgbdp)*trotx(rgbdpo(1))*troty(0)*trotz(0)
p=[0;0;1;1];
pc=T*p
                             
pause(.3);

% Plan a path from the youBot's current position to map cell 30, 20.
goal = [ -20, 10 ];


res = vrep.simxSynchronous(h.id, true); vrchk(vrep, res);
t = 0;
  fsm='rotate';
while true,
  [res youbotPos] = vrep.simxGetObjectPosition(h.id, h.ref, -1,...
                                     vrep.simx_opmode_buffer); vrchk(vrep, res);
  [res youbotEuler] = vrep.simxGetObjectOrientation(h.id, h.ref, -1,...
                                     vrep.simx_opmode_buffer); vrchk(vrep, res);
 
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  forwBackVel = 0;
  leftRightVel = 0;
  rotVel = 0;
	target(2)=-3;%xy(goal(2))
	target(1)=-3;%xy(goal(1))
    target(3)=1.57;
    if strcmp(fsm, 'rotate'),
		P_ow=[target(1);target(2);1];
		T_cw=se2(youbotPos(1),youbotPos(2),youbotEuler(3));
		P_oc=inv(T_cw)*P_ow;  %get the object position relative to the youbot_center coordinate
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
		
	    rotVel =25*angl-10*(-youbotEuler(3)-angl+target(3));

        fsm = 'drive';
% 		if abs(angdiff(angl, youbotEuler(3))) < 1/180*pi,
%             rotVel = 0;
%         end
% 		forwBackVel=0;
   elseif strcmp(fsm, 'drive'),
        forwBackVel = sign(P_oc(2))*20*sqrt((youbotPos(1)-target(1))^2+(youbotPos(2)-target(2))^2);
		%forwBackVel=1000*forwBackVel;
        fsm = 'rotate';
%         if abs(forwBackVel) < .001,
%             forwBackVel = 0;
%         end
%         
% 		 rotVel=0;
         if sqrt((youbotPos(1)-target(1))^2+(youbotPos(2)-target(2))^2)<0.01
             fsm='finished';
         end
    elseif strcmp(fsm, 'finished'),
        break;
    end

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
  t = t+timestep;
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

