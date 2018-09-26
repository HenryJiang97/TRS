function [forwBackVel rotVel]=gotopoint(target,youbotPos_x,youbotPos_y,youbotEuler)

global fsm;

  if strcmp(fsm, 'rotate'),
		P_ow=[target(1);target(2);1];
		T_cw=se2(youbotPos_x,youbotPos_y,youbotEuler);
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
			angl=0;
		end
		
	    rotVel =10*angl;%angdiff(angl, youbotEuler(3));
        %youbotEuler(3);
        fsm = 'drive';
		if abs(angdiff(angl, youbotEuler)) < 1/180*pi,
            rotVel = 0;
        end
		forwBackVel=0;
   elseif strcmp(fsm, 'drive'),
        forwBackVel = 20*((youbotPos_x-target(1))^2+(youbotPos_y-target(2))^2);
		%forwBackVel=1000*forwBackVel
        if abs(forwBackVel) < .001,
            forwBackVel = 0;
        end
         fsm = 'rotate';
		 rotVel=0;
   end
   
   
end