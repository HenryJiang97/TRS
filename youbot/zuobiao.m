




        res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi/8,...
                                 vrep.simx_opmode_oneshot_wait);
        vrchk(vrep, res);
        
        res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1,...
                                   vrep.simx_opmode_oneshot_wait);
        vrchk(vrep, res);
       
        pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait); 
 
       res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1,...
                                   vrep.simx_opmode_oneshot_wait);
         vrchk(vrep, res);
        fprintf('Capturing image...\n');





[res, resolution, image] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    
    subplot(224)
        imshow(image);
        drawnow;
    
    
        a=double(image);
        modeblue=(a(:,:,1)>150).*
                 (a(:,:,2)>150).*
                 (a(:,:,3)<100);
          a(:,:,1)=255*modeblue;
          a(:,:,2)=255*modeblue;
          a(:,:,3)=255*modeblue;

figure,imshow(uint8(a)); 

b=rgb2gray(a);
bw=im2bw(b);
[r c]=find(bw==1);   
[rectx,recty,area,perimeter] = minboundrect(c,r,'a'); 
line(rectx(:),recty(:),'color','r');


center = ceil([(rectx(1,1)+rectx(2,1))/2,(recty(2,1)+recty(3,1))/2]);
center_real = (center - 256)/1600;
center_real(2)=-center_real(2);
 
zs = find((pts(1,:)-center_real(1))<0.005& ...
           (pts(1,:)-center_real(1))>-0.005& ...
           (pts(2,:)-center_real(2))< 0.005& ...
           (pts(2,:)-center_real(2))>-0.005);
       
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
            
            fsm = 'extend';
        elseif strcmp(fsm, 'extend')
            [res, tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
            vrchk(vrep, res, true);