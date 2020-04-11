function [next,closestObstacle,direction,v] = trajfuzzykolejny(app,current,direction,goal,inputObstacles,max_turning,ro,gotowe,obstacleType,a,vo,discrete_time, takingVelocityIntoAccount)
% Computes new position basing on parameters usingfuzzy logic, it returns new position, direction, value of velocity and the distance to closest obstacle

	% Firslty we set parameters
	dist_x=goal(1)-current(1);
	dist_y=goal(2)-current(2);
	dist_z=goal(3)-current(3);
	dist_poziom = sqrt(dist_x^2+dist_y^2);
	dist = sqrt(dist_x^2+dist_y^2+dist_z^2);
	angleVertcal=direction(1);
	angleHorizontal=direction(2);
	roz = size(inputObstacles);
	type = roz(1);
	closestObstacle=ro;
	vmax=vo+a;
	vmin=vo-a;
	% Computing velocity to goal
	angleHorizontalGoal=atan2(dist_y,dist_x);

	angleVerticalGoal=atan2(dist_z,dist_poziom);
	toGoalHorizontal=(angleHorizontalGoal-angleVertcal)*discrete_time;
	toGoalHorizontal = normakat(app,toGoalHorizontal,max_turning);
	toGoalVertical=(angleVerticalGoal-angleHorizontal)*discrete_time;
	toGoalVertical = normakat(app,toGoalVertical,max_turning);
	vToGoal=min(min(max(a,(1.5*a*dist)),1),dist);
	%  Computing velocity to dodge obstacles
	noticedObstacles=0;
	slowngParameter=3;
	for i=1:type %For each obstacle
		if (obstacleType=="kula") %Checing if obstacle move 
			vektorp=inputObstacles(i,:);
		elseif (obstacleType=="uav")
			vektorp=[i,app.minodlegomidzyuavEditField.Value,inputObstacles(i,:)];
		end
			% Computing values such distand and direction to obstacle
			dist_iteration=ro;
			dist_xp=vektorp(1,3)-current(1);
			dist_yp=vektorp(1,4)-current(2);
			dist_zp=vektorp(1,5)-current(3);
			dist_p = sqrt(dist_xp^2+dist_yp^2+dist_zp^2) - vektorp(1,2);
			dist_help=(dist_xp^2+dist_yp^2+dist_zp^2);
			dist_xp = dist_xp*sqrt(abs(dist_p)/dist_help);
			dist_yp = dist_yp*sqrt(abs(dist_p)/dist_help);
			dist_zp = dist_zp*sqrt(abs(dist_p)/dist_help);
			dist_poziomp = sqrt(dist_xp^2+dist_yp^2);
			angleVertcalp=atan2(dist_yp,dist_xp);
			angleHorizontalp=atan2(dist_zp,dist_poziomp);
			% If angle is bigger then 0.2/lower than -0.2, we have to make it 0.2/-0.2 because bigger/lower value cannot be input of our fuzzy system
			toObstacleHorizontal = angleVertcalp-angleVertcal ;
			toObstacleHorizontal = normakat(app,toObstacleHorizontal,pi/2);
			
			if ((toObstacleHorizontal>=0)&&(toObstacleHorizontal<0.2))
				toObstacleHorizontal=0.2;
			end
			if ((toObstacleHorizontal<0)&&(toObstacleHorizontal>-0.2))
				toObstacleHorizontal=-0.2;
			end
			toObstacleVertical = angleHorizontalp-angleHorizontal ;
			toObstacleVertical = normakat(app,toObstacleVertical,pi/2);
			if ((toObstacleVertical>=0)&&(toObstacleVertical<0.2))
				toObstacleVertical=0.2;
			end
			if ((toObstacleVertical<0)&&(toObstacleVertical>-0.2))
				toObstacleVertical=-0.2;
			end
			wallObstacle=current+[dist_xp dist_yp dist_zp];
			if (dist_p<ro) % If we notied obstacle
				if ((czybierzemypoduwage(app,current,vo,direction,wallObstacle,ro,ro/2))||(takingVelocityIntoAccount)) % Computing best velocities for each obstacle 
				dist_iteration=dist_p;
				closestObstacle=min(dist_p,closestObstacle);
				noticedObstacles = noticedObstacles + 1;
				input = [dist_p,toObstacleHorizontal,toObstacleVertical];
				output = evalfis(input,gotowe); % Using fuzzy inference system
				changes(noticedObstacles,1)=output(1);
				changes(noticedObstacles,2)=output(2)*discrete_time;
				changes(noticedObstacles,3)=output(3)*discrete_time;
				changes(noticedObstacles,4)=dist_iteration;
				end
			end
		   
		
	end
	if (noticedObstacles>0) %Situaiotn when robot noticed at least one obstacle
		vToDodge = min(changes(1:noticedObstacles,1));
		v= min(vToGoal,vToDodge);
		slowngParameter=min(changes(1:noticedObstacles,4));
		dodgeHorizontal=sum(changes(1:noticedObstacles,2));
		dodgeHorizontal=normakat(app,dodgeHorizontal,max_turning);
		dodgeVertical=sum(changes(1:noticedObstacles,3));
		dodgeVertical=normakat(app,dodgeVertical,max_turning);
		
	else % There were no obstacles
		v= vToGoal;
		dodgeHorizontal=0;
		dodgeVertical=0;
		
	end
	if (slowngParameter<0)
	   slowngParameter=0; 
	end
	
	% Computing next position
	
	toGoalHorizontal=toGoalHorizontal*(slowngParameter/3)^(1/6);
	toGoalVertical=toGoalVertical*(slowngParameter/3)^(1/6);
	% goal kodu na przeskody
	angleVertcal = angleVertcal +toGoalHorizontal + dodgeHorizontal;
	angleHorizontal = angleHorizontal+toGoalVertical + dodgeVertical;
	
	v=max(min(v,vmax),vmin);
	vx=v*cos(angleHorizontal)*cos(angleVertcal);
	vy=v*cos(angleHorizontal)*sin(angleVertcal);
	vz=v*sin(angleHorizontal);
	next(1) = current(1) + vx;
	next(2) = current(2) + vy;
	next(3) = current(3) + vz;
	direction=[angleVertcal,angleHorizontal];
     
end