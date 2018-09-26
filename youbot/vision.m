   close all;

   disp('Program started');
   %Use the following line if you had to recompile remoteApi
   %vrep = remApi('remoteApi', 'extApi.h');
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
   
   % This will only work in "continuous remote API server service"
   % See http://www.v-rep.eu/helpFiles/en/remoteApiServerSide.htm
   res = vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);
   % We're not checking the error code - if vrep is not run in continuous remote
   % mode, simxStartSimulation could return an error.
   % vrchk(vrep, res);
   
   % Retrieve all handles, and stream arm and wheel joints, the robot's pose,
   % the Hokuyo, and the arm tip pose.
   h = youbot_init(vrep, id);
   h = youbot_hokuyo_init(vrep, h);
   
   vrep.simxSetObjectOrientation(id, h.rgbdCasing, h.ref, [0 0 pi * 0.85], vrep.simx_opmode_oneshot);
   pause(.2)
   
    res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi / 8, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    
   res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1,...
                                   vrep.simx_opmode_oneshot_wait);
   vrchk(vrep, res);
   fprintf('Capturing point cloud...\n');
   pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);

   
   res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1,...
                                   vrep.simx_opmode_oneshot_wait);
         vrchk(vrep, res);
   fprintf('Capturing image...\n');

   [res, resolution, image] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);

imshow(image);

a = double(image);
modepurple=zeros(size(a(:,:,1)));
modeyellow=(a(:,:,1)>150).*(a(:,:,2)>150).*(a(:,:,3)>150);
a(:,:,1)=255*modeyellow;
a(:,:,2)=255*modeyellow;
a(:,:,3)=255*modeyellow;
figure,imshow(uint8(a));

b=rgb2gray(a);
bw=im2bw(b);
[r c]=find(bw==1);   
[rectx,recty,area,perimeter] = minboundrect(c,r,'a'); 
line(rectx(:),recty(:),'color','r');

center = ceil([(rectx(1,1)+rectx(2,1))/2,(recty(2,1)+recty(3,1))/2]);

center_real = (center - 256)/1600;
center_real(2)=-center_real(2);
zs = find((pts(1,:)-center_real(1))<0.05& ...
           (pts(1,:)-center_real(1))>-0.05& ...
           (pts(2,:)-center_real(2))< 0.05& ...
           (pts(2,:)-center_real(2))>-0.05); 
       
    z = sum(pts(3,zs))/size(zs,2);
    y = sum(pts(2,zs))/size(zs,2);
    x = sum(pts(1,zs))/size(zs,2);

[res, rgbdp]=vrep.simxGetObjectPosition(h.id, h.xyzSensor, h.armRef,vrep.simx_opmode_buffer);  
[res, rgbdpo]=vrep.simxGetObjectOrientation(h.id, h.xyzSensor, h.armRef, vrep.simx_opmode_buffer);  

T =transl(rgbdp)*trotx(rgbdpo(1))*troty(rgbdpo(2))*trotz(rgbdpo(3));
 pos_l = T * [x;y;z;1];
 pos_l = [-3;-5.65;0.2451;1];
 
 





