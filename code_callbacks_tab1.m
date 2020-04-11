function WyczyButtonPushed(app, event) % Deletes 1st plot
	app.UIAxes.cla;
end

function StworzprzeszkodyButtonPushed(app, event) %Create random obstacles
	app.Przeszkody= stworzprzeszkody(app,app.Srty_2.Value);
	app.UITable.Data=app.Przeszkody;
	
end

function UITableCellEdit(app, event) %Save data from table in global varaible (reading global variable is faster than reading table)
	app.Przeszkody=app.UITable.Data;
end

function NarysujprzeszkodyButtonPushed(app, event) % Draw all saved obstacles on the plot 
	obstacles= rysujprzeszkody(app,app.Przeszkody);
	for i=1:length(obstacles) % Draw each obstacle
		q= obstacles{i};
		if (length(q)>0);
			[x,y,z] = sphere;
			surf(app.UIAxes,app.Przeszkody(i,2)*x+app.Przeszkody(i,3),app.Przeszkody(i,2)*y+app.Przeszkody(i,4),app.Przeszkody(i,2)*z+app.Przeszkody(i,5))
			hold(app.UIAxes,'on')
		end

	end
	%Set limits
	xlim(app.UIAxes,[0 11])
	ylim(app.UIAxes,[0 11])
	zlim(app.UIAxes,[0 11])
end
     
	 
function pokapozycjeButtonPushed(app, event) % Shows robot's position in the chosen moment on the plot and its closest distance to obstalce in that moment if their positions are drawn 
	% prepare a plot
	app.UIAxes.cla
	xlim(app.UIAxes,[0 11])
	ylim(app.UIAxes,[0 11])
	zlim(app.UIAxes,[0 11])
	% Draw robot
	help=size(app.droga);
	if (app.drogaEditField.Value>help(1))
		app.drogaEditField.Value=help(1)
	end
	path2=app.droga{1,app.drogaEditField.Value}
	position = min(app.EditField.Value,length(path2))
	plot3(app.UIAxes,path2(position,1),path2(position,2),path2(position,3),'*')
	theClosest = 100
	if (app.NarysowaprzeszkodyCheckBox.Value) % If user wants also to draw obstacles, obstacles are drawn
	    sizeof = size(app.Przeszkody)
		for i=1:sizeof(1)
			[x,y,z] = sphere;
			surf(app.UIAxes,app.Przeszkody(i,2)*x+app.Przeszkody(i,3)+app.Przeszkody(i,6)*(position-1),app.Przeszkody(i,2)*y+app.Przeszkody(i,4)+app.Przeszkody(i,7)*(position-1),app.Przeszkody(i,2)*z+app.Przeszkody(i,5)+app.Przeszkody(i,8)*(position-1))
			hold(app.UIAxes,'on')
			distance=(((app.Przeszkody(i,3)+app.Przeszkody(i,6)*(position-1))-path2(position,1))^2+((app.Przeszkody(i,4)+app.Przeszkody(i,7)*(position-1))-path2(position,2))^2+((app.Przeszkody(i,5)+app.Przeszkody(i,8)*(position-1))-path2(position,3))^2)
			theClosest=min(theClosest,distance)
		end
	end
	app.najbliejEditField.Value = theClosest %show the closest distance to obstacle
end

function KolejnaiteracjaButtonPushed(app, event) % Similar to pokapozycjeButtonPushed, but draws the position which is chosen amount of iterations later or ealier. Clicking rapidly shows how robot was moving
% prepare a plot
	app.UIAxes.cla
	xlim(app.UIAxes,[0 11])
	ylim(app.UIAxes,[0 11])
	zlim(app.UIAxes,[0 11])
	app.EditField.Value = app.EditField.Value + app.EditField_2.Value
	% Draw robot
	help=size(app.droga);
	if (app.drogaEditField.Value>help(1))
		app.drogaEditField.Value=help(1)
	end
	path2=app.droga{1,app.drogaEditField.Value}
	position = min(app.EditField.Value,length(path2))
	plot3(app.UIAxes,path2(position,1),path2(position,2),path2(position,3),'*')
	theClosest = 100
	if (app.NarysowaprzeszkodyCheckBox.Value)% If user wants also to draw obstacles, obstacles are drawn
	   sizeof = size(app.Przeszkody)
		for i=1:sizeof(1)
				[x,y,z] = sphere;
			   surf(app.UIAxes,app.Przeszkody(i,2)*x+app.Przeszkody(i,3)+app.Przeszkody(i,6)*(position-1),app.Przeszkody(i,2)*y+app.Przeszkody(i,4)+app.Przeszkody(i,7)*(position-1),app.Przeszkody(i,2)*z+app.Przeszkody(i,5)+app.Przeszkody(i,8)*(position-1))
				 hold(app.UIAxes,'on')
				 distance=(((app.Przeszkody(i,3)+app.Przeszkody(i,6)*(position-1))-path2(position,1))^2+((app.Przeszkody(i,4)+app.Przeszkody(i,7)*(position-1))-path2(position,2))^2+((app.Przeszkody(i,5)+app.Przeszkody(i,8)*(position-1))-path2(position,3))^2)
				 theClosest=min(theClosest,distance)
		end
	end
	app.najbliejEditField.Value = theClosest%show the closest distance to obstacle
end
      
function ZapisButtonPushed(app, event) % Save data from first table in the chosen directory
	p=app.UITable.Data;
	road=strcat(app.EditField_5.Value,'.mat');
	save(road,'p')
end

function WczytajButtonPushed(app, event) % Load data to first table in the chosen directory
	road=strcat(app.EditField_5.Value,'.mat');
	try
	load(road)
	app.UITable.Data=p; 
	app.Przeszkody=p;
	catch
	end
end

function LosujButtonPushed(app, event) % Create random starting position and destination in 3-D space
	app.Srtx_2.Value = rand()*10;
	app.Srty_3.Value = rand()*10;
	app.Srtz_2.Value = rand()*10;
	app.kncx_2.Value = rand()*10;
	app.kncy_2.Value = rand()*10;
	app.kncz_2.Value = rand()*10;
end

function WykresButtonPushed(app, event) % Create plot which shows how close robot was to obstacle in each iteration
	path3=app.droga{1,app.drogaEditField.Value}
	close=app.droga{2,app.drogaEditField.Value}
	for i=1:length(path3) % For each moment of time
	   dystansgoal(i)=dystans(app,path3(i,:),path3(end,:))
	   kolejno(i)=i; 
	end
	% Clear plot and set parameters
	cla(app.UIAxes);
	plot(app.UIAxes,dystansgoal,kolejno);
	hold(app.UIAxes,'on');
	plot(app.UIAxes,[close 3],kolejno);
	hold(app.UIAxes,'off');
	rotate3d(app.UIAxes,'on');
	view(app.UIAxes,-90,90)
end

function DodajprzeszkodyButtonPushed(app, event) %Adding row to table
	obstacles=app.UITable.Data;
	if (isempty(obstacles))
		obstacles=0;
	end
	help=max(obstacles(:,1));
	for i=(help+1):(help+app.Srty_4.Value)
	   obstacles(i,1)=i;
	   obstacles(i,2)=5;
	   obstacles(i,3)=5;
	   obstacles(i,4)=5;
	   obstacles(i,5)=5;
	end
	app.UITable.Data=obstacles;
end

function UsuwierszButtonPushed(app, event) %Earsing row from 1st table
	c=app.UITable.Data;
	row=size(c);
	help=0;
	try % There is error when table is empty 
		for i=1:row(1)
			 if (i~=app.Srty_5.Value)
				 help=help+1;
				 for j=1:row(2)
					 if (j==1)
						d(help,j)=help;
					 else
						d(help,j)=c(i,j);
					 end
				 end
			 end
		end
		app.UITable.Data=d;
	catch %If table is empty, do nothing
	end
end

function PokazdrogeButtonPushed(app, event) % Draw trajetory with chosen color o the first plot
	color = app.colorDropDown.Value;
	if (app.DropDown.Value=="punkty")
		color=color+"*";
	end
	path3=app.droga{1,app.drogaEditField.Value}
	plot3(app.UIAxes,path3(:,1),path3(:,2),path3(:,3),color);
	hold(app.UIAxes,'on');
	rotate3d(app.UIAxes,'on');
	app.DystansEditField.Value = trasa(app,path3);
	app.iterationEditField.Value = length(path3);
	app.najbliejEditField.Value =app.droga{2,min(app.drogaEditField.Value)};
end

function OblicztrajektorieButtonPushed(app, event) % After clicking button, simulation in 1st tab is started
	% Check starting position, obstacles etc.
	start = [app.Srtx.Value, app.Srty.Value, app.Srtz.Value]
	obstacles= app.Przeszkody;
	goal = [app.kncx.Value, app.kncy.Value, app.kncz.Value];
	ro=app.WykrywanieprzeszkdSpinner.Value;
	% Set parameters
	timeAll=0;
	timeMax=0;
	timeIteration=1/app.iterationnasekundEditField.Value
	% Choose method
	if (app.Metoda.Value=="potential fields")
		% Set parameters
		v=0;
		index=1;
		path3=start;
		theClosest=100;
		distance_x=goal(1)-start(1);
		distance_y=goal(2)-start(2);
		distance_z=goal(3)-start(3);
		distance = sqrt(distance_x^2+distance_y^2+distance_z^2);
		distance_horizontal = sqrt(distance_x^2+distance_y^2);
		angleHorizontal=atan2(distance_y,distance_x);
		angleVertical=atan2(distance_z,distance_horizontal);
		direction=[angleHorizontal,angleVertical];
		while ((distance>0.1)||(v>0.1)) % the simulation continues until the robot reaches its destination
			timeHelp=tic; % Cheching real time
			index=index+1; % Checking iteration	  
		    current=path3(index-1,:); % Saving entire trajectory in variable
			[current,close,direction,v] = trajkolejny(app,current,goal,obstacles,app.WzmocnienieprzyciganiaEditField.Value,app.WzmocnienieodpychaniaEditField.Value,ro,'kula',v,app.MaksymalneprzypieszenieEditField.Value*timeIteration,direction,app.turningMaxEditField.Value*timeIteration,app.NiebracpoduwagewlasnejpredkosciCheckBox.Value); % Computing next velocity
			current=path3(end,:)+(current-path3(end,:))*timeIteration; % Computing position basing on velocity and discretization time
			path3=[path3;current]; %Savin new position
			distance = dystans(app,current,goal); %Checking how close to destination robot is
		    theClosest(index-1)=close; % Sabing the closest distance to obstacle
			% Checking real time parameters
		    time=toc(timeHelp); %
			timeMax=max(time,timeMax);
			timeAll=timeAll+time;
		end
		close=min(theClosest); % Checking the closest distnce to obstacle in the entire simulation
	end
	% Choose method
	if (app.Metoda.Value=="shortest road")
		% Set parameters
		v=0;
		index=1;
		path3=start;
		theClosest=100;
		distance_x=goal(1)-start(1);
		distance_y=goal(2)-start(2);
		distance_z=goal(3)-start(3);
		distance_horizontal = sqrt(distance_x^2+distance_y^2);
		distance = sqrt(distance_x^2+distance_y^2+distance_z^2);
		angleHorizontal=atan2(distance_y,distance_x);
		angleVertical=atan2(distance_z,distance_horizontal);
		direction=[angleHorizontal,angleVertical];
		road=[];
		previousObstacles=[];
		while ((distance>0.1)||(v>0.1)) % the simulation continues until the robot reaches its destination
			timeHelp=tic; % Cheching real time
			index=index+1; % Checking iteration	   
			current=path3(index-1,:); % Saving entire trajectory in variable
			[current,close,direction,v,road,previousObstacles] = geometria(app,current,direction,goal,obstacles,previousObstacles,app.turningMaxEditField.Value*timeIteration,ro,road,"kula",app.MaksymalneprzypieszenieEditField.Value*timeIteration,v,app.DropDown_2.Value,app.NiebracpoduwagewlasnejpredkosciCheckBox.Value,app.minodlegomidzyuavEditField.Value);% Computing next velocity
			current=path3(end,:)+(current-path3(end,:))*timeIteration; % Computing position basing on velocity and discretization time
			path3=[path3;current]; %Savin new position
			distance = dystans(app,current,goal); %Checking how close to destination robot is
		    theClosest(index-1)=close; % Sabing the closest distance to obstacle
			% Checking real time parameters
			time=toc(timeHelp);
			timeMax=max(time,timeMax);
			timeAll=timeAll+time;
		end
		close=min(theClosest); % Checking the closest distnce to obstacle in the entire simulation
	end
	% Choose method
	if (app.Metoda.Value=="logika rozmyta")
		% Set parameters
		v=0
		app.fis.defuzzMethod = app.OdrozmywanieDropDown.Value;
		app.fis.andMethod   = app.metodaandDropDown.Value;
		index=1;
		path3=start;
		theClosest=100;
		distance_x=goal(1)-start(1);
		distance_y=goal(2)-start(2);
		distance_z=goal(3)-start(3);
		distance_horizontal = sqrt(distance_x^2+distance_y^2);
		distance = sqrt(distance_x^2+distance_y^2+distance_z^2);
		angleHorizontal=atan2(distance_y,distance_x);
		angleVertical=atan2(distance_z,distance_horizontal);
		direction=[angleHorizontal,angleVertical];
		while ((distance>0.1)||(v>0.1)) % the simulation continues until the robot reaches its destination
			timeHelp=tic; % Cheching real time
			index=index+1; % Checking iteration	   
			current=path3(index-1,:); % Saving entire trajectory in variable
			[current,close,direction,v] = trajfuzzykolejny(app,current,direction,goal,obstacles,app.turningMaxEditField.Value*timeIteration,ro,app.fis,"kula",app.MaksymalneprzypieszenieEditField.Value*timeIteration,v,timeIteration,app.NiebracpoduwagewlasnejpredkosciCheckBox.Value);% Computing next velocity
			current=path3(end,:)+(current-path3(end,:))*timeIteration; % Computing position basing on velocity and discretization time
			path3=[path3;current]; %Savin new position
			distance = dystans(app,current,goal); %Checking how close to destination robot is
		    theClosest(index-1)=close; % Sabing the closest distance to obstacle
			% Checking real time parameters
			time=toc(timeHelp);
			timeMax=max(time,timeMax);
			timeAll=timeAll+time;
		end
		close=min(theClosest); % Checking the closest distnce to obstacle in the entire simulation
	end
	if (app.NarysowaprzeszkodyCheckBox.Value) % Obstacle can be drawn 
		sizeof = size(obstacles)
		for i=1:sizeof(1) % Amount of pbstacles
			[x,y,z] = sphere;
		    surf(app.UIAxes,app.Przeszkody(i,2)*x+app.Przeszkody(i,3),app.Przeszkody(i,2)*y+app.Przeszkody(i,4),app.Przeszkody(i,2)*z+app.Przeszkody(i,5));
			hold(app.UIAxes,'on');
		end
	end
	color = app.colorDropDown.Value; % Checking which color user wants
	if (app.DropDown.Value=="punkty")
		color=color+"*";
	end
	% Drawing trajectory
	plot3(app.UIAxes,path3(:,1),path3(:,2),path3(:,3),color);
	hold(app.UIAxes,'on');
	% Showing outputs in the aplication
	app.DystansEditField.Value = trasa(app,path3);
	app.iterationEditField.Value = length(path3);
	app.imaksymalnyEditField.Value=timeMax;
	app.CzasobliczeredniEditField.Value=timeAll/length(path3);
	app.najbliejEditField.Value =close;
	app.indexdroga=app.indexdroga+1;
	app.droga{1,app.indexdroga}=path3;
	app.droga{2,app.indexdroga}=theClosest;
end