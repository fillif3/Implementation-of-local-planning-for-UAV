function [force_x,force_y,force_z,dist] = force_ujemna(app,current_position,obstacle,kr,ro)
	% Compute negative force to one obstacle
	dist_x=obstacle(3)-current_position(1);
	dist_y=obstacle(4)-current_position(2);
	dist_z=obstacle(5)-current_position(3);
	dist = sqrt(dist_x^2+dist_y^2+dist_z^2) - obstacle(2); % Dist to obstacle
	% Dist to obstacle in x,y and z
	dist_x = dist_x*dist/sqrt(dist_x^2+dist_y^2+dist_z^2);
	dist_y = dist_y*dist/sqrt(dist_x^2+dist_y^2+dist_z^2);
	dist_z = dist_z*dist/sqrt(dist_x^2+dist_y^2+dist_z^2);

	if (dist>ro) %If we are not close to it, it is 0
		force_x=0;
		force_y=0;
		force_z=0;
	else %otherwise, we use equesion for negative force of artificial potential fields
		force_x=(-kr*dist_x/(dist^3))*((1/dist)-(1/ro));
		force_y=(-kr*dist_y/(dist^3))*((1/dist)-(1/ro));
		force_z=(-kr*dist_z/(dist^3))*((1/dist)-(1/ro));
	end
end

function [force_x,force_y,force_z] = force_dodatnia(app,current_position,dotargetowe,ka) 
	%Compute positive force
	dist_x=dotargetowe(1)-current_position(1);
	dist_y=dotargetowe(2)-current_position(2);
	dist_z=dotargetowe(3)-current_position(3);
	dist_horizontal = sqrt(dist_x^2+dist_y^2);
	dist = dystans(app,dotargetowe,current_position);
	if (dist<1) % if we are close, positive force rises
		force_x = ka*dist_x;
		force_y = ka*dist_y;
		force_z = ka*dist_z;
	else % otherwise, only direction changes
		sinus_vertical = dist_z/dist ;
		cosinus_vertical= dist_horizontal/dist ;
		sinus_horizontal=dist_x/dist_horizontal;
		cosinus_horizontal=dist_y/dist_horizontal;
		
		force_x=ka*cosinus_vertical*sinus_horizontal;
		force_y=ka*cosinus_vertical*cosinus_horizontal;
		force_z=ka*sinus_vertical;
	end
    
end

function [next,closestObstacle,direction,v] = trajkolejny(app,current,goal,vektorp,ka,kr,ro,obstacleType,vo,a,direction,max_turning,takingVelocityIntoAccount)
% Computes new position basing on parameters using Artificial potential fields, it returns new position, direction, value of velocity and the distance to closest obstacle

	% Firslty we set parameters
	dist = dystans(app,current,goal)
	roz = size(vektorp)
	type = roz(1)
	closestObstacle=ro;
	vmax=vo+a;
	vmin=vo-a;
	% Computing positive force
	[force_px,force_py,force_pz] = force_dodatnia(app,current,goal,ka);
	force_msx=0;
	force_msy=0;
	force_msz=0;
	% For each obstacle
	for i=1:type
		%Checking if obstacle move or not
		if (obstacleType=="kula")
			obstacles=vektorp(i,:);
		elseif (obstacleType=="uav")
			obstacles=[i,app.minodlegomidzyuavEditField.Value,vektorp(i,:)];
		end
			% Computing negative force
			[force_mx,force_my,force_mz,dist_p] = force_ujemna(app,current,obstacles,kr,ro);
			% Saving dist to closest obstacle
			closestObstacle=min(dist_p,closestObstacle);
			dist_xp=obstacles(1,3)-current(1);
			dist_yp=obstacles(1,4)-current(2);
			dist_zp=obstacles(1,5)-current(3);
			dist_p = sqrt(dist_xp^2+dist_yp^2+dist_zp^2) - vektorp(1,2);
			dist_help=(dist_xp^2+dist_yp^2+dist_zp^2);
			dist_xp = dist_xp*sqrt(abs(dist_p)/dist_help);
			dist_yp = dist_yp*sqrt(abs(dist_p)/dist_help);
			dist_zp = dist_zp*sqrt(abs(dist_p)/dist_help);
			wallObstacle=current+[dist_xp dist_yp dist_zp];
				% We can still ignore obstacle if we choose diffrent method of taking the into account
				if ((czybierzemypoduwage(app,current,vo,direction,wallObstacle,ro,ro/2))||(takingVelocityIntoAccount))
				    % Summing all negative forces
					force_msx=force_msx+force_mx;
					force_msy=force_msy+force_my;
					force_msz=force_msz+force_mz;
				end
	end
	force_x=force_px+force_msx;
	force_y=force_py+force_msy;
	force_z=force_pz+force_msz;
	% We choose bew valocity basing on force and parametrs
	vgoal=min(sqrt(force_x^2+force_y^2+force_z^2),1);
	v=max(min(vgoal,vmax),vmin);
	% Trying to slove problem of local minimum
	if ((v<0.1)&&(v>dist))
	   v=0.1;
	   v=max(min(v,vmax),vmin);
	end
	if ((force_y==0)&&(force_x==0)) % If we have velocity equal to zero we cannot compute direction in this iteration so we use previous direction 
		angleHorizontalGoal=direction(1);
	else
		angleHorizontalGoal=atan2(force_y,force_x);
	end
	if ((force_y==0)&&(force_x==0)&&(force_z==0)) %Same as before
		angleVerticalGoal=direction(2);
	else
		angleVerticalGoal=atan2(force_z,sqrt(force_x^2+force_y^2));
	end
	% Computing next position
	toGoalHorizontal=angleHorizontalGoal-direction(1);
	toGoalHorizontal = normakat(app,toGoalHorizontal,max_turning);
	toGoalVertical=angleVerticalGoal-direction(2);
	toGoalVertical = normakat(app,toGoalVertical,max_turning);
	angleVertcal=direction(1)+toGoalHorizontal;
	angleHorizontal=direction(2)+toGoalVertical;
	vx=v*cos(angleHorizontal)*cos(angleVertcal);
	vy=v*cos(angleHorizontal)*sin(angleVertcal);
	vz=v*sin(angleHorizontal);
	next(1) = current(1) + vx;
	next(2) = current(2) + vy;
	next(3) = current(3) + vz;
	direction=[angleVertcal,angleHorizontal];
end