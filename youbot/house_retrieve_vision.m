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

%pickupJoints = [120*pi/180, 120*pi/180, 120*pi/180, 120*pi/180, 180*pi/180];


timestep = .05;
T=20;
N_step=T/timestep;

[tposx,tposxd,tposxdd]=tpoly(-3,-3,N_step);
[tposy,tposyd,tposydd]=tpoly(-5.6,-5.65,N_step);
[tposz,tposzd,tposzdd]=tpoly(0.2101,0.2451,N_step);
tpos=[tposx tposy tposz];

[tposx,tposxd,tposxdd]=tpoly(-3,-2.8,N_step);
[tposy,tposyd,tposydd]=tpoly(-5.65,-5.3,N_step);
[tposz,tposzd,tposzdd]=tpoly(0.2451,0.4,N_step);
tpos1=[tposx tposy tposz];

[tposx,tposxd,tposxdd]=tpoly(-6.6133,-6.815,N_step);
[tposy,tposyd,tposydd]=tpoly(-2.4343,-2.87,N_step);
[tposz,tposzd,tposzdd]=tpoly(0.4,0.3,N_step);
tpos_dog=[tposx tposy tposz];


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

goal = [ ij(-5.26), ij(-3.25) ];
dx = DXform(double(map), 'private');
dx.plan([goal(2); goal(1)]);

%Map
figure(2), dx.plot(); figure(1);

traj = dx.path([ij(youbotPos(1)) ; ij(youbotPos(2))])';

traj = [ xy(traj(1,:)); xy(traj(2,:)) ];

save('traj', 'traj');


res = vrep.simxSynchronous(h.id, true); vrchk(vrep, res);
t = 0;

T=20;
N_step=T/timestep;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Main code(Calculation)

fsm = 'rotate';

while true,
    [res youbotPos] = vrep.simxGetObjectPosition(h.id, h.ref, -1,...
                                     vrep.simx_opmode_buffer); vrchk(vrep, res);
    [res youbotEuler] = vrep.simxGetObjectOrientation(h.id, h.ref,...
                                 -1, vrep.simx_opmode_buffer); vrchk(vrep, res);   
    
    target = traj(:,1);
  
    if size(traj, 2) > 1 & sqrt((youbotPos(1) - traj(1,1))^2 + (youbotPos(2) - traj(2,1))^2) < 0.5
        traj = traj(:,2:end);
    end
    
    angl = pi/2;
    if strcmp(fsm, 'rotate'),
        rotVel = 10*angdiff(angl, youbotEuler(3));
        if abs(angdiff(angl, youbotEuler(3))) < 1/180*pi,
            rotVel = 0;
            fsm = 'drive';
        end

    
    elseif strcmp(fsm, 'drive'),
            forwBackVel = 10 * sqrt((youbotPos(1) - target(1))^2 + (youbotPos(2) - target(2))^2)
            
            fsm = 'rotate';
            
            if (youbotPos(1) - target(1)) < 0.1
                
                i = 1;
                forwBackVel = 0;
                rotVel = 0;

                fsm = 'snap';
            end 	
            
        elseif strcmp(fsm, 'snap'),
        
            res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi/4,...
                                 vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
                
            res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1,...
                                   vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
       
            pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait); 
 
            res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1,...
                                   vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
                
            [res, resolution, image] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
                
            
            
            
            a=double(image);
            modeyellow=(a(:,:,1)>150).* (a(:,:,2)>150).* (a(:,:,3)<100);
            a(:,:,1)=255*modeyellow;
            a(:,:,2)=255*modeyellow;
            a(:,:,3)=255*modeyellow;
            
            b=rgb2gray(a);
            bw=im2bw(b);
            [r c]=find(bw==1);   
            [rectx,recty,area,perimeter] = minboundrect(c,r,'a'); 
            line(rectx(:),recty(:),'color','r');
            
            center = ceil([(rectx(1,1)+rectx(2,1))/2,(recty(2,1)+recty(3,1))/2]);
            center_real = (center - 256)/1600;
            center_real(2)=-center_real(2);
 
            zs = find((pts(1,:)-center_real(1))<0.005 & (pts(1,:)-center_real(1))>-0.005 & (pts(2,:)-center_real(2))< 0.005 & (pts(2,:)-center_real(2))>-0.005);
       
            z = sum(pts(3,zs))/size(zs,2);
            y = sum(pts(2,zs))/size(zs,2);
            x = sum(pts(1,zs))/size(zs,2);
            
            [res youbotPos] = vrep.simxGetObjectPosition(h.id, h.ref, -1,...
                                                 vrep.simx_opmode_buffer); vrchk(vrep, res);
            [res youbotEuler] = vrep.simxGetObjectOrientation(h.id, h.ref,...
                                                 -1, vrep.simx_opmode_buffer); vrchk(vrep, res);

            [res rgbdp]=vrep.simxGetObjectPosition(h.id, h.xyzSensor, h.armRef, vrep.simx_opmode_buffer);  
            [res rgbdpo]=vrep.simxGetObjectOrientation(h.id, h.xyzSensor, h.armRef, vrep.simx_opmode_buffer);  

            T =transl(rgbdp)*trotx(rgbdpo(1))*troty(rgbdpo(2))*trotz(rgbdpo(3));
            
            pos_l = T * [x;y;z;1];
            
            tic;  
            for t = 0.001:0.001:2  
                while toc < t  
                end  
            end
            disp(pos_1)
            
            fsm = 'extend';

        
        elseif strcmp(fsm, 'extend'),
            
            res = vrep.simxSetIntegerSignal(id, 'km_mode', 2,...
                                   vrep.simx_opmode_oneshot_wait);
            
            res = vrep.simxSetObjectPosition(id, h.ptarget, -1, tpos(i,:), vrep.simx_opmode_oneshot);
            
            i=i+1;
            if i<N_step
                fsm='extend';
            else
                fsm='grasp';
                pause(1);
                i=1;
            end
    elseif strcmp(fsm, 'grasp'),
        res = vrep.simxSetIntegerSignal(id, 'gripper_open', 0,...
                                   vrep.simx_opmode_oneshot_wait);
                               
        vrchk(vrep, res);
        pause(1);
        
        i = 1;
        fsm = 'raise';
        
    elseif strcmp(fsm, 'raise'),
        res = vrep.simxSetObjectPosition(id, h.ptarget, -1, tpos1(i,:), vrep.simx_opmode_oneshot);
        i = i + 1;
        
        if i<N_step
            fsm='raise';
        else
            fsm='rotate_dog';
            pause(1);
            i=1;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            [res youbotPos] = vrep.simxGetObjectPosition(h.id, h.ref, -1,...
                                     vrep.simx_opmode_buffer); vrchk(vrep, res);
            [res youbotEuler] = vrep.simxGetObjectOrientation(h.id, h.ref, -1,...
                                     vrep.simx_opmode_buffer); vrchk(vrep, res);
                             
            pause(.3);

            goal = [ ij(-2.5), ij(-6.9) ];
            dx = DXform(double(map), 'private');
            dx.plan([goal(2); goal(1)]);
            
            traj = dx.path([ij(youbotPos(1)) ; ij(youbotPos(2))])';

            traj = [ xy(traj(1,:)); xy(traj(2,:)) ];

            save('traj', 'traj');
            
        end
        
    elseif strcmp(fsm, 'rotate_dog'),
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

        fsm = 'drive_dog';


    elseif strcmp(fsm, 'drive_dog'),
        forwBackVel = 10 * sqrt((youbotPos(1) - target(1))^2 + (youbotPos(2) - target(2))^2)

        fsm = 'rotate_dog';

        if sqrt((youbotPos(1) - target(1))^2 + (youbotPos(2) - target(2))^2) < 0.03
            %pause(5);
            fsm='throw_dog';
            i = 1;
            forwBackVel = 0;
            rotVel = 0; 
        end
    
    elseif strcmp(fsm, 'throw_dog'),
        res = vrep.simxSetObjectPosition(id, h.ptarget, -1, tpos_dog(i,:), vrep.simx_opmode_oneshot);
            
        i=i+1;
        
        if i<N_step
            fsm='throw_dog';
        else
            fsm='release_dog';
            pause(1);
            i = 1;
        end
    
    elseif strcmp(fsm, 'release_dog'),
        res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1,...
                                   vrep.simx_opmode_oneshot_wait);
                               
        vrchk(vrep, res);
        pause(5);
        
        i = 1;
        pause(5);
        fsm = 'finished';

        
    elseif strcmp(fsm, 'finished'),
        
        ;
        break;
           
    end
   
    

    
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