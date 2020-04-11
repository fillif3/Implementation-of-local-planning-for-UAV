function WyczyButton_2Pushed(app, event) % Deletes 2nd plot
	app.UIAxes_2.cla;
end


function pokapozycjeButton_2Pushed(app, event) % Draw positions of all robots in the chosem moment of time
	% Set parameters
	app.UIAxes_2.cla
	xlim(app.UIAxes_2,[0 11])
	ylim(app.UIAxes_2,[0 11])
	zlim(app.UIAxes_2,[0 11])
	if (app.EditField_3.Value>length(app.zbiortras{1})) % If moving was already finished, show last positions
		app.EditField_3.Value=length(app.zbiortras{1})
	end
	% Draw them
	for i=1:length(app.zbiortras)
		path2=app.zbiortras{i};
		color=string(app.UITable_2.Data(i,end))+'*';
		plot3(app.UIAxes_2,path2(app.EditField_3.Value,1),path2(app.EditField_3.Value,2),path2(app.EditField_3.Value,3),color);
		hold(app.UIAxes_2,'on');
	end
end

function KolejnaiteracjaButton_2Pushed(app, event) % Similar to KolejnaiteracjaButtonPushed, but position for more than one robot is shown
% prepare a plot
	app.UIAxes_2.cla
	xlim(app.UIAxes_2,[0 11])
	ylim(app.UIAxes_2,[0 11])
	zlim(app.UIAxes_2,[0 11])
	% Which moment of time
	app.EditField_3.Value = app.EditField_3.Value + app.EditField_4.Value
	if (app.EditField_3.Value>length(app.zbiortras{1}))
		app.EditField_3.Value=length(app.zbiortras{1})
	end
	for i=1:length(app.zbiortras) % Draw each robot
		path2=app.zbiortras{i};
		color=string(app.UITable_2.Data(i,end))+'*';
		plot3(app.UIAxes_2,path2(app.EditField_3.Value,1),path2(app.EditField_3.Value,2),path2(app.EditField_3.Value,3),color);
		hold(app.UIAxes_2,'on');
	end
end

function ZapisButton_2Pushed(app, event) % Save data from second table in the chosen directory
	p=app.UITable_2.Data;
	road=strcat(app.EditField_5.Value,'.mat');
	save(road,'p')
end

function WczytajButton_2Pushed(app, event) % Load data to second table in the chosen directory
	try
	road=strcat(app.EditField_5.Value,'.mat');
	load(road)
	app.UITable_2.Data=p; 
	catch
	end
end

% Button pushed function: UsunwierszButton
function UsunwierszButtonPushed(app, event) %Earsing row from 2nd table
	c=app.UITable_2.Data;
	row=size(c);
	help=0;
	try % There is error when table is empty 
		for i=1:row(1)
			 if (i~=app.EditField_7.Value)
				 help=help+1;
				 for j=1:row(2)
					 if (j==1)
						d{help,j}=help;
					 else
						d{help,j}=c{i,j}
					 end
				 end
			 end
		end
		app.UITable_2.Data=d;
	catch %If table is empty, do nothing
	end
	
end

function DodajobiektButtonPushed(app, event) % Add a new robot with information descirbied in inputs 
	% Save table in data
	data = app.UITable_2.Data;              
	app.UITable_2.Data={lp,app.Srtx_2.Value,app.Srty_3.Value,app.Srtz_2.Value,app.kncx_2.Value,app.kncy_2.Value,app.kncz_2.Value,app.Metoda_2.Value,help,app.colorDropDown_2.Value}; 
	if (isempty(data)) % If empty, it is first row
		lp=1
	else
		a=cell2mat(data(:,1)); %otherwise, it isnext row
		lp=max(a)+1;
	end
	C = data;
	% add data to row
	C{lp,1}=lp;
	C{lp,2}=app.Srtx_2.Value;
	C{lp,3}=app.Srty_3.Value;
	C{lp,4}=app.Srtz_2.Value;
	C{lp,5}=app.kncx_2.Value;
	C{lp,6}=app.kncy_2.Value;
	C{lp,7}=app.kncz_2.Value;
	C{lp,8}=app.Metoda_2.Value;
	C{lp,9}=app.WykrywanieprzeszkdSpinner_2.Value;
	C{lp,10}=app.turningMaxEditField_2.Value;
	C{lp,11}=app.MaksymalneprzypieszenieEditField_2.Value;
	C{lp,14}=app.NiebracpoduwagewlasnejpredkosciCheckBox_2.Value;
	% Some parameters are dependet on type of algorithm
	if (C{lp,8}=="potential fields")
		C{lp,12}=app.WzmocnienieprzyciganiaEditField_2.Value;
		C{lp,13}=app.WzmocnienieodpychaniaEditField_2.Value;
	elseif (C{lp,8}=="logika rozmyta")
		C{lp,12}=app.OdrozmywanieDropDown_2.Value;
		C{lp,13}=app.DropDown_4.Value;
	else
		C{lp,12}=app.DropDown_3.Value;
		C{lp,13}=app.BezpiecznaodlegoEditField_2.Value;
	end
	
	C{lp,20}=app.colorDropDown_2.Value;
	
	% Save new table
	app.UITable_2.Data=C;
	
end


function OblicztrajektorieButton_2Pushed(app, event) % After clicking button, simulation in 2nd tab is started
	% Setting parameters
	data = app.UITable_2.Data;
	lp=max(cell2mat(data(:,1)));
	pathList={};
	for i=1:lp % For each robot, set parameters
		ro(i)=data{i,9};
		turningMax(i)=data{i,10};
		amax(i)=data{i,11};
		start=[data{i,2},data{i,3},data{i,4}]
		pathList{i}=start;
		goal(i,1)=data{i,5};
		goal(i,2)=data{i,6};
		goal(i,3)=data{i,7};
		timeMax(i)=0;
		timeAll(i)=0;
		iteration(i)=0;
		theClosest(i)=inf;
		stillWorking(i)=1;
		angleHorizontal=atan2(goal(i,2)-start(2),goal(i,1)-start(1))
		angleVertical=atan2(goal(i,3)-start(3),sqrt((goal(i,1)-start(1))^2+(goal(i,2)-start(2))^2))
		direction(i,1)=angleHorizontal;
		direction(i,2)=angleVertical;
		v(i)=0;
		niebracpoduwageswojejpredkosci(i)=data{i,14};
		if (data{i,8}=="potential fields") %Some parameters depend on type of used algorithm
			ka(i)=data{i,12};
			kp(i)=data{i,13};
		elseif (data{i,8}=="shortest road")
			odstep(i)=data{i,13};
			sciezki{i}=[];
			previousObstacles{i}=[];
		end
	end
	index=1;% Iteration
	flag=0;
	timeIterationstandard=1/app.iterationnasekundEditField_2.Value;
    while (sum(stillWorking)) % while at least on robot is not at its destination
	    obstacles=[];
	    for i=1:lp
			% save current positions as obstacles
		   uav=pathList{i};
		   obstacles(i,1)=uav(index,1);
		   obstacles(i,2)=uav(index,2);
		   obstacles(i,3)=uav(index,3);
	    end
	    index=index+1;
	    for i=1:lp % for each robot
			if (stillWorking(i)) % if robot moves
				next=pathList{i}; % curent position
				current=next(index-1,:);
				currentObstacles=obstacles;
				currentObstacles(i, :)=[]; % current robot is not obstacle
				% Depending on chosen algorithm, next position and velocity is comupted
			    if (data{i,8}=="potential fields")
					timeHelp=tic;
					[current,close,direction(i,:),v(i)] = trajkolejny(app,current,goal(i,:),currentObstacles,ka(i),kp(i),ro(i),'uav',v(i),amax(i)*timeIterationstandard,direction(i,:),turningMax(i)*timeIterationstandard,niebracpoduwageswojejpredkosci(i));
					timeIteration=toc(timeHelp);
					timeMax(i)=max(timeMax(i),timeIteration);
					timeAll(i)=timeIteration+timeAll(i);
					theClosest(i)=min(theClosest(i),close);
			    elseif (data{i,8}=="logika rozmyta")
					app.fis.defuzzMethod=data{i,12};
					timeHelp=tic;
					[current,close,direction(i,:),v(i)] = trajfuzzykolejny(app,current,direction(i,:),goal(i,:),currentObstacles,turningMax(i)*timeIterationstandard,ro(i),app.fis,'uav',amax(i)*timeIterationstandard,v(i),timeIterationstandard,niebracpoduwageswojejpredkosci(i));
					timeIteration=toc(timeHelp);
					timeMax(i)=max(timeMax(i),timeIteration);
					timeAll(i)=timeIteration+timeAll(i);
					theClosest(i)=min(theClosest(i),close);
			    else
				    if(flag) % flag is used because in this algorithm, robot uses previous position of obstacles. in th first iteration, it would try to use index out of range
						timeHelp=tic;
						[current,close,direction(i,:),v(i),sciezki{i},previousObstacles{i}] = geometria(app,current,direction(i,:),goal(i,:),currentObstacles,previousObstacles{i},turningMax(i)*timeIterationstandard,ro(i),sciezki{i},"uav",amax(i)*timeIterationstandard,v(i),'astar',niebracpoduwageswojejpredkosci(i),odstep(i));
						timeIteration=toc(timeHelp);
						timeMax(i)=max(timeMax(i),timeIteration);
						timeAll(i)=timeIteration+timeAll(i);
						theClosest(i)=min(theClosest(i),close);
				    else
					    previousObstacles{i}=currentObstacles;
					  
				    end
			    end
				% Save robot's information
			    current=next(end,:)+(current-next(end,:))*timeIterationstandard;
			    next=[next;current];
			    pathList{i}=next;
			    distance = dystans(app,current,goal(i,:));
			    if (distance<0.1)
			    	stillWorking(i)=0;
			    end
		    else % robot's position is not chenged otherwise
			    if (iteration(i)==0)
				    iteration(i)=index;
			    end
			    next=pathList{i};
			    next=[next;next(index-1,:)];
			    pathList{i}=next;
		    end
	    end
	    flag=1;
    end
    for i=1:lp % for each robot, save outputs in table and draw trajectory
	   color=string(data{i,end})+'*';
	   path3=pathList{i};
	   plot3(app.UIAxes_2,path3(:,1),path3(:,2),path3(:,3),color);
	   hold(app.UIAxes_2,'on');
	   app.UITable_2.Data{i,15}=trasa(app,path3);
	   app.UITable_2.Data{i,16}=theClosest(i);
	   if (iteration(i)==0)
		  app.UITable_2.Data{i,17}=index;
		  iteration(i)=index;
	   else
			app.UITable_2.Data{i,17}=iteration(i);
	   end
	   app.UITable_2.Data{i,18}=timeAll(i)/iteration(i);
	   app.UITable_2.Data{i,19}=timeMax(i);
    end
    rotate3d(app.UIAxes_2,'on');
    app.zbiortras=pathList;
end