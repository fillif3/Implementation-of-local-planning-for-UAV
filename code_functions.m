% List of functions and explenations used in the code



function angle = normaangle(app,x,limit)
% It takes angle x in radians, then we look for value <-pi,pi> which has the same output for cosinus and sinus as input. (1)
% Then, if input is lower or higher that -limit and limit respectively, the output is equal to limit (or -limit), output is x otherwise (2)
    %(1)
	if (x>pi)
        x = x - 2*pi;
    end
    if (x<-pi)
        x = x + 2*pi;
    end
    %(2)
    angle = max(min(x,limit),-limit);
    
end
		
function dist = dystans(app,current,target)
% Finds the distance between current position and target position in 3-D space
	dist = sqrt((target(1)-current(1))^2+(target(2)-current(2))^2+(target(3)-current(3))^2);
end

function out = distRoad(app,road)
% Similar to previous function,, but find distance for entire road
    q = size(road);
    out=0;
    for i=1:(q-1)
        out = out+sqrt((road(i,1)-road(i+1,1))^2+(road(i,2)-road(i+1,2))^2+(road(i,3)-road(i+1,3))^2);
                
    end
            
end       
		
% Code that executes after component creation
function startupFcn(app)
	rotate3d(app.UIAxes,'on');
	app.testButton.Visible='off';
	app.DropDown_2.Visible='off';
	app.DropDown_4.Visible='off';
	app.Button.Visible='off';
	app.PrognozaCheckBox_2.Visible='off';
	app.DropDown_3.Visible='off';
	app.NiebracpoduwagewlasnejpredkosciCheckBox_2.Visible='off';
end


function ButtonPushed(app, event) %Saving image
	fig = figure;
	fig.Visible = 'off';
	figAxes = axes(fig);
	% Copy all UIAxes children, take over axes limits and aspect ratio.            
	allChildren = app.UIAxes_2.XAxis.Parent.Children;
	copyobj(allChildren, figAxes)
	figAxes.XLim = app.UIAxes_2.XLim;
	figAxes.YLim = app.UIAxes_2.YLim;
	figAxes.ZLim = app.UIAxes_2.ZLim;
	figAxes.View  = app.UIAxes_2.View ;
	figAxes.DataAspectRatio = app.UIAxes_2.DataAspectRatio;
	% Save as png and fig files.
	saveas(fig, 'qweeqweqweqweqw', 'png');
	savefig(fig, 'qweeqweqweqweqw2323');
	% Delete the temporary figure.
	delete(fig);
end
