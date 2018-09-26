close all;

disp('Program started');
% Use the following line if you had to recompile remoteApi
%vrep = remApi('remoteApi', 'extApi.h');
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);

% If you get an error like: 
%   Remote API function call returned with error code: 64. Explanation: simxStart was not yet called.
% Make sure your code is within a function! You cannot call V-REP from a script. 

if id < 0
disp('Failed connecting to remote API server. Exiting.');
vrep.delete();
return;
end
fprintf('Connection %d to remote API server open.\n', id);

% Make sure we close the connection whenever the script is interrupted.
cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));

% This will only work in "continuous remote API server service". 
% See http://www.v-rep.eu/helpFiles/en/remoteApiServerSide.htm
vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);

[res VisionSensor] = vrep.simxGetObjectHandle(id, 'Vision_sensor', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
[res VisionSensor0] = vrep.simxGetObjectHandle(id, 'Vision_sensor0', vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);

[res, resolution, image] = vrep.simxGetVisionSensorImage2(id, VisionSensor, 0, vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);

%imshow(image);


Image_R=image;
RP_R=Image(:,:,1); 
RP_G=Image(:,:,2); 
RP_B=Image(:,:,3);
XYR=~((R-G)>diff_R&(R-B)>diff_R); 
figure,imshow(uint8(XYR));

Image_G=image;
GP_R=Image(:,:,1); 
GP_G=Image(:,:,2); 
GP_B=Image(:,:,3);
XYG=~((G-R)>diff_G&(G-B)>diff_G); 
figure,imshow(uint8(XYG));

Image_B=image;
BP_R=Image(:,:,1);
BP_G=Image(:,:,2);
BP_B=Image(:,:,3);
XYB=~((B-R)>diff_B&(B-G)>diff_B); 
figure,imshow(uint8(XYB));
