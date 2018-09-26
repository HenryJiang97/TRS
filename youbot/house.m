function house(displayMap)

disp('Program started');
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
id=vrep.simxStart('127.0.0.1',19997,true,true,2000,5);
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

%initial function
h = youbot_init(vrep, id);
timestep = .05;
  %小车轮子的半径
  r = 0.0957;
  %小车轮距半径
  b = 0.1655;
  %小车速度
  forwBackVel = 0;
  rotvel = 0;
  pause(1);
  

  % One map cell is 25cm large
global cellsize;
cellsize = .25;

pause(.3);
  forwBackVel = 0;
  leftRightVel = 0;
  rotVel = 0;
t = 0;
    res = vrep.simxSynchronous(h.id, true); vrchk(vrep, res);


fsm='rotate';
while true,
  [res youbotPos] = vrep.simxGetObjectPosition(h.id, h.ref, -1,...
                                     vrep.simx_opmode_buffer); vrchk(vrep, res);
  [res youbotEuler] = vrep.simxGetObjectOrientation(h.id, h.ref, -1,...
                                     vrep.simx_opmode_buffer); vrchk(vrep, res);
 
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  

	target(1)=5;
	target(2)=5;
    if strcmp(fsm, 'rotate'),
        fsm 
		P_ow=[target(1);target(2);1];
		T_cw=se2(youbotPos(1),youbotPos(2),youbotEuler(3));
		P_oc=inv(T_cw)*P_ow;  %get the object position relative to the youbot_center coordinate
		if P_oc(1)<0 && P_oc(2)<0 
			angle=atan(P_oc(2)/P_oc(1));
			angl=angle-pi;   
        end
        if P_oc(1)<0 && P_oc(2)>0 
			angle=atan(P_oc(2)/P_oc(1));
			angl=angle+pi;   
		end
		if P_oc(1)>0
			angle=atan(P_oc(2)/P_oc(1));
			angl=angle;
		end
		if P_oc(1)==0 && P_oc(2)>0 
			angl=pi/2;
        end
        if P_oc(1)==0 && P_oc(2)<0 
			angl=-pi/2;
        end
        if P_oc(2)==0 
            angl=0;
        end
		angl;
	    rotVel =5*angl;

        fsm = 'drive';
% 		if abs(angdiff(angl, youbotEuler(3))) < 1/180*pi,
%             rotVel = 0;
%         end
% 		forwBackVel=0;
   elseif strcmp(fsm, 'drive'),
        forwBackVel =0.1*sqrt((youbotPos(1)-target(1))^2+(youbotPos(2)-target(2))^2);
		%forwBackVel=1000*forwBackVel;
        fsm = 'rotate';
%         if abs(forwBackVel) < .001,
%             forwBackVel = 0;
%         end
%         
% 		 rotVel=0;
         if sqrt((youbotPos(1)-target(1))^2+(youbotPos(2)-target(2))^2)<0.1
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

  
end
    
%     vLeft = (v - b * w) / r
%     vRight = (v + b * w) / r
%     vrep.simxSetJointTargetVelocity(id, wheelJoints(1),vLeft,vrep.simx_opmode_oneshot); vrchk(vrep, res);
%     vrep.simxSetJointTargetVelocity(id, wheelJoints(2),vRight,vrep.simx_opmode_oneshot); vrchk(vrep, res);
%     %pause(2);

disp('Program ended');
end

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


