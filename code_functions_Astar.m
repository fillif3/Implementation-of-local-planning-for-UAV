function direction = prostaprostopodla(app,inputPoint,inputDirection,point)
	%Finds surface which contains straight line and is perpendicular to line which contains point and is perpendicular to stright line
	% Firstly, we find perpendicular surface (it has same parameters A,B and C as input Direction). Then, we compute parameter D with input point
	D=-(inputDirection(1)*point(1)+inputDirection(2)*point(2)+inputDirection(3)*point(3));
	% Then we find cross point which cross between surface and input line
	t=-(D+inputDirection(1)*inputPoint(1)+inputDirection(2)*inputPoint(2)+inputDirection(3)*inputPoint(3))/(inputDirection(1)^2+inputDirection(2)^2+inputDirection(3)^2);
	crossPoint(1)=inputPoint(1)+inputDirection(1)*t;
	crossPoint(2)=inputPoint(2)+inputDirection(2)*t;
	crossPoint(3)=inputPoint(3)+inputDirection(3)*t;
	# If we have two points, we can compute direction
	direction=point-crossPoint;
	# Compute D
	direction(4)=-(direction(1)*inputPoint(1)+direction(2)*inputPoint(2)+direction(3)*inputPoint(3));
end

function [crossPoint] = pointxplaszczyznaxprosta(app,inputPoint,inputDirection,direction)
% Finds the cross point of line and surface
	t=-(direction(4)+direction(1)*inputPoint(1)+direction(2)*inputPoint(2)+direction(3)*inputPoint(3))/(direction(1)*inputDirection(1)+direction(2)*inputDirection(2)+direction(3)*inputDirection(3));
	crossPoint(1)=inputPoint(1)+inputDirection(1)*t;
	crossPoint(2)=inputPoint(2)+inputDirection(2)*t;
	crossPoint(3)=inputPoint(3)+inputDirection(3)*t;
end

function [dist] = distprostapoint(app,inputPoint,inputDirection,point)
% Finds lowest dist between point and line
	D=-(inputDirection(1)*point(1)+inputDirection(2)*point(2)+inputDirection(3)*point(3));
	t=-(D+inputDirection(1)*inputPoint(1)+inputDirection(2)*inputPoint(2)+inputDirection(3)*inputPoint(3))/(inputDirection(1)^2+inputDirection(2)^2+inputDirection(3)^2);
	crossPoint(1)=inputPoint(1)+inputDirection(1)*t;
	crossPoint(2)=inputPoint(2)+inputDirection(2)*t;
	crossPoint(3)=inputPoint(3)+inputDirection(3)*t;
	dist=dystans(app,point,crossPoint);
end

function listCon = sprpoloczeniaplaszczyzna(app,listPoints,inputObstacles,previousObstacles)
	%Creates graph from list of points and mapd.
    % Check conection between points if there is no UAV between them, the connection is saved as [[p1 index,p2 index, distance betwenn them]...]
    listCon=[];
    knownObstacles=listPoints(2,4); %First obstacle
    index=1;
    for i=3:(length(listPoints)-1) %save obstacles to new variable
        if (knownObstacles(index)~=listPoints(i,4))
            index=index+1;
            knownObstacles(index)=listPoints(i,4); 
        end
    end
    for i=1:(length(listPoints)-1) %for each pair of points
        for j=(i+1):length(listPoints)
            if (listPoints(i,4)~=listPoints(j,4))
                cross=0;
                for k=1:length(knownObstacles) %Check if connection cross with another UAV
                    cross=czyomijacuav(app,[listPoints(i,1:3);listPoints(j,1:3)],inputObstacles(knownObstacles(k),:),previousObstacles(knownObstacles(k),:),0.8);
                            
                    if (cross)
                        break;
                    end
                            
                end
                if (cross==0) %If not, save onnection
                    dist=dystans(app,listPoints(i,:),listPoints(j,:));
                    listCon=[listCon;i j dist];
                end
                        
            end
                    
        end
    end
            
end

function out = stworzprzeszkody(app,count)
% Create random sphere obstalces. Each one needs position (x,y,z) and radius 
	flag= 1;
	out=[];
	for i=1:count % We want to have 'count' obstacles
		while(flag) % We create obstacles until we have obstacle which is not close to being outside 
		   x = rand()*6+2 ;
		   y = rand()*6+2 ;
		   z = rand()*6+2  ;
		   r = rand()*1 ;
		   if (((r+0.5) <x^2+y^2+z^2)&&((r+0.5) <(x-10)^2+(y-10)^2+(z-10)^2))
			   flag = 0;
		   end
		end
		flag = 1;
	   out = [out;i,r,x,y,z];
	end
end

function directional=rowanieprostej(app,current,goal)
	% Finds directional coefficients of the straight line basing on two points
	dif=goal-current;
	dist=dystans(app,current,goal);
	directional=dif/dist;
end

function y=czyprzecina(app,current,destination,obstacle,r)
	% Check if line between two points cross with obstacle or is closer than r to obstacle
    directional=rowanieprostej(app,current,destination);
	parameter=(directional*(current'-obstacle'))^2-dystans(app,current,obstacle)^2+r^2;
	if (parameter<=0) %If infinity straight line does not cross sphere
		y=0;
	else % Otherwse, check if cross point is between start and end points
		roz1=sqrt(parameter)-(directional*(current'-obstacle'));
		roz2=-sqrt(parameter)-(directional*(current'-obstacle'));
		distance=(destination(1)-current(1))/directional(1);
		if (((abs(distance)>abs(roz1))&&(sign(distance)==sign(roz1)))||((abs(distance)>abs(roz2))&&(sign(distance)==sign(roz1))))
			y=1;
		else
			y=0;
		end
	end

end

function out = stworzpunkty(app,current,vektorp)
	% Creates points around sphere, basing on actual the current's position
    directional=rowanieprostej(app,current,vektorp(3:5));
    if (directional(3)==0) % We don want to divide by 0
        directional(3)=0.0001
    end
    directional=directional/directional(3)
    a=directional(1);
    b=directional(2);
    c=directional(3);
    m=vektorp(3);
    n=vektorp(4)
    k=vektorp(5);
    r=vektorp(2);
    nr=linspace(vektorp(1),vektorp(1),8) %obstacle ordinal number

	% 8 points around obstacle are created
	x(1)=sqrt((r^2)/(a^2+1));
	y(1)=0;
	z(1)=-a*x(1)-b*y(1)+k;
	x(1)=x(1)+m;
	y(1)=y(1)+n
			
	x(2)=-sqrt((r^2)/(a^2+1));
	y(2)=0;
	z(2)=-a*x(2)-b*y(2)+k;
	x(2)=x(2)+m;
	y(2)=y(2)+n;
	
	x(3)=0;
	y(3)=sqrt((r^2)/(b^2+1));
	z(3)=-a*x(3)-b*y(3)+k;
	x(3)=x(3)+m;
	y(3)=y(3)+n
	
	x(4)=0;
	y(4)=-sqrt((r^2)/(b^2+1));;
	z(4)=-a*x(4)-b*y(4)+k;
	x(4)=x(4)+m;
	y(4)=y(4)+n;
	
	x(5)=sqrt((r^2)/(a^2+2*a*b+b^2+2));
	y(5)=x(5);
	z(5)=-a*x(5)-b*y(5)+k;
	x(5)=x(5)+m;
	y(5)=y(5)+n
	
	x(6)=sqrt((r^2)/(a^2-2*a*b+b^2+2));
	y(6)=-x(6);
	z(6)=-a*x(6)-b*y(6)+k;
	x(6)=x(6)+m;
	y(6)=y(6)+n
	
	x(7)=-sqrt((r^2)/(a^2+2*a*b+b^2+2));
	y(7)=x(7);
	z(7)=-a*x(7)-b*y(7)+k;
	x(7)=x(7)+m;
	y(7)=y(7)+n
	
	x(8)=-sqrt((r^2)/(a^2-2*a*b+b^2+2));
	y(8)=-x(8);
	z(8)=-a*x(8)-b*y(8)+k;
	x(8)=x(8)+m;
	y(8)=y(8)+n
			

	out=[x',y',z',nr'];
end

function listConnections = sprpoloczenia(app,listPoints,obstacles)
	%Creates graph from list of points and mapd.
    % Check conection between points if there is no obstacle between them, the connection is saved as [[p1 index,p2 index, distance betwenn them]...]
	listConnections=[];
	knownObstacles=listPoints(2,4); %We check only know obtacles
	index=1;
	for i=3:(length(listPoints)-1)
		if (knownObstacles(index)~=listPoints(i,4))
		   index=index+1;
		   knownObstacles(index)=listPoints(i,4);
		end
	end
	for i=1:(length(listPoints)-1)
	   for j=(i+1):length(listPoints) %Check every pair of points
			if (listPoints(i,4)~=listPoints(j,4))
				cross=0;
				for k=1:length(knownObstacles) % Check connection for every obstacle
					cross=czyprzecina(app,listPoints(i,1:3),listPoints(j,1:3),obstacles(knownObstacles(k),3:5),obstacles(knownObstacles(k),2));
					if (cross)
						break;
						end
					
				end
				if (cross==0) %If save, we add it list of connections
					distance=dystans(app,listPoints(i,:),listPoints(j,:));
					listConnections=[listConnections;i j distance];
				end
				
			end
			
	   end
	end
	
end

function out = dikstrja(app,listpkt,listConnections)
    % Args: list_of_points - matrix of points -> [[x1,y1],[x2,y2]... [xn,yn]] where n = amount of points and each point is described by x and y (last point is goal and first is start)
    %  list_connects - list of possible connection between points - each row is one connection and first two coulmns are ordinal number of points and 3r column is the distance
	% Using Dijkstra algorithm
	m=true;
	n=true;
	countPointEnd=1;
	countCon=length(listConnections);
	countPoint=max(listConnections(:,2));
	maximum=countPoint
	
	for i=2:countPoint
	   road(i,1)=inf; % We create graph according to algorithm
	   road(i,3)=0;
    % graph - in this matrix we have information about each point (row per point) in columns we have
    % 1st - shortest possible road found until now
    % 2nd - if this place was already considered - if yes: 1 else: 0
    % 3rd - from which  point the shortest path was found (ordinal number)
	end
	where=0; % Actually considered point
	while(n)
	   distance=inf; 
	   for i=1:countPoint %We look for the best point
		   if ((road(i,1)<distance)&&(road(i, 2) == 0))
				distance=road(i,1);
				where = i;
		   end
	   end
	   road(where,2)=1;%After finding point, we write that was already considered so we won't choose it again
	   if (where == maximum)%if we consider goal point
		  while(m)
			 countPointEnd= countPointEnd+1; %We check how many point will road have
			  where=road(where,3);
			  if(where==1)
				  m=0;%After findind shortest road, we leave loop
				  n=0;
			  end
			  
		  end
		   where=maximum
	   else
		   for i=1:countCon
				if ((listConnections(i, 1) == where) && ((listConnections(i, 3) + road(where, 1)) < road(listConnections(i, 2), 1)))%looking for connections with considered point BUT only to the point which has longer path than sum of path to the considered point and distance to the connected point
					road(listConnections(i, 2), 3) = where; %we save considered point in the graph
					road(listConnections(i, 2), 1)  = listConnections(i, 3) + road(where, 1);%we save distance
				end
				%Same but for the 2nd column of list_connects
				if ((listConnections(i, 2) == where) && ((listConnections(i, 3) + road(where, 1)) < road(listConnections(i, 1), 1)))
					road(listConnections(i, 1), 3) = where; 
					road(listConnections(i, 1), 1)  = listConnections(i, 3) + road(where, 1);
				end
		   end
	   end
	end
	for i=1:countPointEnd
		order(countPointEnd-i+1)=where;
		where=road(where, 3);
	end
	% in loop we look for the order of points
	out=[];
	for i=1:countPointEnd
		out=[out;listpkt(order(i),1:3)];
	end
end

function out = astar(app,listpkt,listConnections)
    % Args: list_of_points - matrix of points -> [[x1,y1],[x2,y2]... [xn,yn]] where n = amount of points and each point is described by x and y (last point is goal and first is start)
    %  list_connects - list of possible connection between points - each row is one connection and first two coulmns are ordinal number of points and 3r column is the distance
	% Using A* algorithm
	m=true;
	n=true;
	countPointEnd=1;
	countCon=length(listConnections);
	countPoint=max(listConnections(:,2));
	maximum=countPoint
	
	for i=2:countPoint
	   road(i,1)=inf; 
	   road(i,3)=0;
	end
	where=0; % Actually considered point
	while(n)
	   distance=inf; 
	   for i=1:countPoint
		   if ((road(i,1)<distance)&&(road(i, 2) == 0))%We look for the best point
				distance=road(i,1)+dystans(app,listpkt(i,1:3),listpkt(end,1:3));
				where = i;
		   end
	   end
	   road(where,2)=1;%After finding point, we write that was already considered so we won't choose it again
	   if (where == maximum)%if we consider goal point
		  while(m)
			 countPointEnd= countPointEnd+1; %We check how many point will road have
			  where=road(where,3);
			  if(where==1)
				  m=0;%After findind shortest road, we leave loop
				  n=0;
			  end
			  
		  end
		   where=maximum
	   else
		   for i=1:countCon
				if ((listConnections(i, 1) == where) && ((listConnections(i, 3) + road(where, 1)) < road(listConnections(i, 2), 1)))%looking for connections with considered point BUT only to the point which has longer path than sum of path to the considered point and distance to the connected point
					road(listConnections(i, 2), 3) = where; %we save considered point in the graph
					road(listConnections(i, 2), 1)  = listConnections(i, 3) + road(where, 1);%we save distance
				end
				%Same but for the 2nd column of list_connects
				if ((listConnections(i, 2) == where) && ((listConnections(i, 3) + road(where, 1)) < road(listConnections(i, 1), 1)))
					road(listConnections(i, 1), 3) = where; 
					road(listConnections(i, 1), 1)  = listConnections(i, 3) + road(where, 1);
				end
		   end
	   end
	end
	for i=1:countPointEnd
		order(countPointEnd-i+1)=where;
		where=road(where, 3);
	end
	out=[];
	for i=1:countPointEnd
		out=[out;listpkt(order(i),1:3)];
	end
end

function y = czyomijacuav(app,road,Obstacle,peviousObstacle,gap)
	% Decides if we should try to avoid moving obstacle basing on its previous and curent position, our road and minimum distance wa want to have
	directionObstacle=Obstacle-peviousObstacle; %Direction of unknown moving obstacle
	countPoints=size(road);
	y=0;
	for i=1:(countPoints(1)-1) %For each pair of points
		direction=road(i+1,:)-road(i,:) % direction of line between them
		directionSurface=prostaprostopodla(app,Obstacle,directionObstacle,road(i,:)); %surface which contains obstacle, its previous position and is perpendicular to line between current position of obstacle  and current first pint of road pair
		crossPoint = punktxplaszczyznaxprosta(app,road(i,:),direction,directionSurface); % look for croos between this surface and line between pair of road points
		if (abs(crossPoint)<inf) % if line and surface are not paralel
			dist = dist_prostapunkt(app,Obstacle,directionObstacle,crossPoint);
			if (dist<gap) %If cross point is closer then minimum distance
				index=1;
				% check if cross point is between two points of road
				xmax=max(road(i,1),road(i+1,1));
				xmin=min(road(i,1),road(i+1,1));
				if (xmax==xmin) %line is parallel to x-axis
				   xmax=max(road(i,2),road(i+1,2));
				   xmin=min(road(i,2),road(i+1,2)); 
				   index=2; 
					if (xmax==xmin) %line is alos parallel to y-axis
					  xmax=max(road(i,3),road(i+1,3));
					   xmin=min(road(i,3),road(i+1,3)); 
					   index=3;   
					   
					end
				end
				if ((crossPoint(index)>xmin)&&(crossPoint(index)<xmax)) % We should try to avoid
				   y=1;
				   break
				end
			
			end
		end
	end
end

function y = stworzpunktyplaszczyzna(app,current,obstacle,directionObstacle,gap)
	% Creates best points to avoid mobile obstacles basing on its previous and curent position, our position and minimum distance wa want to have
	directionSurface=prostaprostopodla(app,obstacle,directionObstacle,current);
	% Firslty we need to find surface oon which we will have points
	drectionPerpendicularToTwoLines(1)=directionObstacle(2)*directionSurface(3)-directionObstacle(3)*directionSurface(2);
	drectionPerpendicularToTwoLines(2)=directionObstacle(3)*directionSurface(1)-directionObstacle(1)*directionSurface(3);
	drectionPerpendicularToTwoLines(3)=directionObstacle(1)*directionSurface(2)-directionObstacle(2)*directionSurface(1);
	directionObstacle=directionObstacle/sqrt(directionObstacle(1)^2+directionObstacle(2)^2+directionObstacle(3)^2);
	drectionPerpendicularToTwoLines=drectionPerpendicularToTwoLines/sqrt(drectionPerpendicularToTwoLines(1)^2+drectionPerpendicularToTwoLines(2)^2+drectionPerpendicularToTwoLines(3)^2);
	% Then we create those points
	y(1)=-directionObstacle(1)*gap+obstacle(1);
	y(2)=-directionObstacle(2)*gap+obstacle(2);
	y(3)=-directionObstacle(3)*gap+obstacle(3);
	helpPoint=punktxplaszczyznaxprosta(app,current,directionSurface,directionSurface);
	distHelp=(helpPoint(1)-obstacle(1))/directionObstacle(1);
	for i=1:3
		dist=distHelp*i/2;
		Point(1)=directionObstacle(1)*dist+obstacle(1);
		Point(2)=directionObstacle(2)*dist+obstacle(2);
		Point(3)=directionObstacle(3)*dist+obstacle(3);
		y(2*i,1)=drectionPerpendicularToTwoLines(1)*gap+Point(1);
		y(2*i,2)=drectionPerpendicularToTwoLines(2)*gap+Point(2);
		y(2*i,3)=drectionPerpendicularToTwoLines(3)*gap+Point(3);
		y(2*i+1,1)=-drectionPerpendicularToTwoLines(1)*gap+Point(1);
		y(2*i+1,2)=-drectionPerpendicularToTwoLines(2)*gap+Point(2);
		y(2*i+1,3)=-drectionPerpendicularToTwoLines(3)*gap+Point(3);
	end
end

function y=czybierzemypoduwage(app,current,v,direction,obstacle,tp,r)
	% Creates elipsoid basing on current velocity. If obstacle is outside, we do not take it into account even when it is noticed
	dp=max(r,tp*v);
	dc=(dp-r)/2;
	angle_vertical=direction(2);
	angle_horizontal=direction(1);
	i(1)=cos(angle_vertical)*cos(angle_horizontal);
	i(2)=cos(angle_vertical)*sin(angle_horizontal);
	i(3)=sin(angle_vertical);
	center=current+dc*i;
	d1=(dp+r)/2;
	d2=sqrt((2*r^3)/(dp+r));
	De=[d1,d2,d2];
	Tr1=[cos(angle_horizontal) sin(angle_horizontal) 0; -sin(angle_horizontal) cos(angle_horizontal) 0; 0 0 1];
	Tr2=[cos(angle_vertical) 0 -sin(angle_vertical); 0 1 0; sin(angle_vertical) 0 cos(angle_vertical)];
	Ts=[1/De(1) 0 0; 0 1/De(2) 0; 0 0 1/De(3)];
	T=Tr1*Tr2*Ts;
	Pat=[0 0 0];
	Pdt=((obstacle-center)*T);
	if (dystans(app,Pat,Pdt)<=1)
		y=1;
	else
		y=0;
	end
end

function [next,closestObstacle,direction,v,road,previousObstacle] = geometria(app,current,direction,goal,inputObstacle,previousObstacle,max_turning,ro,road,obstacleType,a,vo,method,takingVelocityIntoAccount,gap)
% Computes new position basing on parameters using global methods, it returns new position, direction, value of velocity and the distance to closest obstacle

	% Firslty we set parameters	
	
	angleHorizontal=direction(1);
	angleVertical=direction(2);
	roz = size(inputObstacle);
	type = roz(1);
	closestObstacle=ro;
	vmax=vo+a;
	vmin=vo-a;
	if (isempty(road)==1) % if we not have road, we go to destination
		target=goal;
	else
	   target=road(1,:) 
	end

	

	cross=0;
	if (obstacleType=="kula") % In this method, we use different algorithm for different types of obstacles
		for i=1:type %For each obstacle
			% compute distance to obstacle
			vektorp=inputObstacle(i,:);
			distance_xp=vektorp(3)-current(1);
			distance_yp=vektorp(4)-current(2);
			distance_zp=vektorp(5)-current(3);
			distance_p = sqrt(distance_xp^2+distance_yp^2+distance_zp^2) - vektorp(2);
			distance_help=(distance_xp^2+distance_yp^2+distance_zp^2);
			distance_xp = distance_xp*sqrt(abs(distance_p)/distance_help);
			distance_yp = distance_yp*sqrt(abs(distance_p)/distance_help);
			distance_zp = distance_zp*sqrt(abs(distance_p)/distance_help);
			if (distance_p<ro) %If obstacle is noticed
				closestObstacle=min(distance_p,closestObstacle); %Remember closest obstacle
				obstacleWall=current+[distance_xp distance_yp distance_zp];
				if ((czybierzemypoduwage(app,current,vo,direction,obstacleWall,ro,ro/2))||(takingVelocityIntoAccount)) %Depending on parameters
					if (~ismember(vektorp(1),previousObstacle)) %If obstacle moved
						 % We check if our current road is safe
						cross=czyprzecina(app,current,target,vektorp(3:5),vektorp(2)+gap);
						sizeRoad=size(road)
						for j=1:(sizeRoad(1)-1) %tutaj wrócić   dsaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
							cross=max(czyprzecina(app,road(j,:),road(j+1,:),vektorp(3:5),vektorp(2)+gap),cross); %dodać bezpieczną odległość
						end
						if (isempty(road)==0)
							cross=max(czyprzecina(app,road(end,:),goal,vektorp(3:5),vektorp(2)+gap),cross); %dodać bezpieczną odległość
						end
						if (cross)
							break %Aktualna ścieżka jest kolizyjna
						else
							previousObstacle=[previousObstacle,vektorp(1)];
						end
					end
				end
			end
		end
		
		if (cross) %If it is not safe
			% We need to find new road
			listPoints=[current,0] 
			for i=1:type % For each obstacle create set of points
				vektorp=inputObstacle(i,:); 
				distance_xp=vektorp(1,3)-current(1);
				distance_yp=vektorp(1,4)-current(2);
				distance_zp=vektorp(1,5)-current(3);
				distance_p = sqrt(distance_xp^2+distance_yp^2+distance_zp^2) - vektorp(1,2);
				if (distance_p<ro)
					vektorp(2)=vektorp(2)+2*gap;%vektorp(2)*1.4)
					newPoints=stworzpunkty(app,current,vektorp);
					listPoints=[listPoints;newPoints];
				end
				
			end
			listPoints=[listPoints;goal (type+1)];
			previousObstacle=listPoints(:,4)';
			listConnections=sprpoloczenia(app,listPoints,inputObstacle); %Find save connections between all points
			if (method=="dikstrja") %Depending on chosen alorithm (Dijkstra of A*), the shortetst path is chosen
				road=dikstrja(app,listPoints,listConnections);
			else
				road=astar(app,listPoints,listConnections);
			end
			road(1,:)=[];
			target=road(1,:);
		end
	else   
		for i=1:type%For each obstacle
			% compute distance to obstacle
			vektorp=inputObstacle(i,:);
			distance_xp=vektorp(1)-current(1);
			distance_yp=vektorp(2)-current(2);
			distance_zp=vektorp(3)-current(3);
			distance_p = sqrt(distance_xp^2+distance_yp^2+distance_zp^2) -app.minodlegomidzyuavEditField.Value;
			distance_help=(distance_xp^2+distance_yp^2+distance_zp^2);
			distance_xp = distance_xp*sqrt(abs(distance_p)/distance_help);
			distance_yp = distance_yp*sqrt(abs(distance_p)/distance_help);
			distance_zp = distance_zp*sqrt(abs(distance_p)/distance_help);
			if (distance_p<ro) %If obstacle is noticed
				closestObstacle=min(distance_p,closestObstacle);%Remember closest obstacle
				obstacleWall=current+[distance_xp distance_yp distance_zp];
				if ((czybierzemypoduwage(app,current,vo,direction,obstacleWall,ro,ro/2))||(app.NiebracpoduwagewlasnejpredkosciCheckBox.Value))%Depending on parameters
					if(~isequal(vektorp,previousObstacle(i,:))) %If obstacle moved
						if (isempty(road)==0)
							cross=czyomijacuav(app,[current;road;target],vektorp,previousObstacle(i,:),gap)  % Check if road is safe
						else
							cross=czyomijacuav(app,[current;target],vektorp,previousObstacle(i,:),gap)  % Check if road is safe
						end
					else
						  cross=0;
					end
					if (cross)
						break 
					end
				end
		    end
		end
		if (cross)%If it is not safe
			% We need to find new road
			listPoints=[current,0]
			for i=1:type % For each obstacle create set of points
				
				vektorp=inputObstacle(i,:); 
			 
				distance_xp=vektorp(1,1)-current(1);
				distance_yp=vektorp(1,2)-current(2);
				distance_zp=vektorp(1,3)-current(3);
				distance_p = sqrt(distance_xp^2+distance_yp^2+distance_zp^2) - app.minodlegomidzyuavEditField.Value;
				if (distance_p<ro) % If obstacle is noticed ]
					%Create new points
					gap=app.minodlegomidzyuavEditField.Value+gap*2;
					newPoints=stworzpunktyplaszczyzna(app,current,vektorp,vektorp-previousObstacle(i,:),gap); 
					nr=linspace(i,i,7);
					newPoints(:,4)=nr;
					listPoints=[listPoints;newPoints];
				end
				
			end
			listPoints=[listPoints;goal (type+1)];
			listConnections=sprpoloczeniaplaszczyzna(app,listPoints,inputObstacle,previousObstacle) % Create graph with safe connection between points
			if (method=="dikstrja") % Choose algorithm to find shortest path
				road=dikstrja(app,listPoints,listConnections);
			else
				road=astar(app,listPoints,listConnections);
			end
			road(1,:)=[];
			target=road(1,:);
		end
		
		previousObstacle=inputObstacle;
	end
	% Compute velocity and next obstacle

	distance_x=target(1)-current(1);
	distance_y=target(2)-current(2);
	distance_z=target(3)-current(3);
	distance_poziom = sqrt(distance_x^2+distance_y^2);
	distance = sqrt(distance_x^2+distance_y^2+distance_z^2);
	app.DystansEditField.Value=distance;
	angleHorizontalTarget=atan2(distance_y,distance_x);
	angleVerticalTarget=atan2(distance_z,distance_poziom);
	toTargetHorizontal=angleHorizontalTarget-angleHorizontal;
	toTargetHorizontal = normakat(app,toTargetHorizontal,max_turning);
	toTargetVertical=angleVerticalTarget-angleVertical;
	toTargetVertical = normakat(app,toTargetVertical,max_turning);
	angleHorizontal=angleHorizontal+toTargetHorizontal;
	angleVertical=angleVertical+toTargetVertical;
	vTurning=((pi-abs(toTargetHorizontal)-abs(toTargetVertical))/pi)^4;
	v=min(min(min(max(a,(1.5*a*distance)),1),vTurning),distance);
	v=max(min(v,vmax),vmin);
	vx=v*cos(angleVertical)*cos(angleHorizontal);
	vy=v*cos(angleVertical)*sin(angleHorizontal);
	vz=v*sin(angleVertical);
	next(1) = current(1) + vx;
	next(2) = current(2) + vy;
	next(3) = current(3) + vz;
	if ((dystans(app,target,next)<0.1)&&(isempty(road)==0))
	   road(1,:)=[]; 
	end
	direction=[angleHorizontal,angleVertical];
end