function plot_occmap(robot)
% plots the occupancy map on robot coordinate

%while 1 
	maxr = 1.0;
	Div = 72;
	Interval = 2*pi/Div;
	HalfInter = Interval/2;
	occumap = {};
	occumap.r = robot.wcmOccumap.get_r();
	occumap.theta = zeros(Div*4,1);
	occumap.rho = zeros(Div*4,1);
	occumap.x = zeros(Div*4,1);
	occumap.y = zeros(Div*4,1);
	for Order = 1 : Div 
		Idx = (Order-1) * 4 + 1;
		midTheta = (Order-1)*Interval;
		occumap.theta(Idx) = midTheta - HalfInter;
		occumap.theta(Idx+1) = midTheta - HalfInter;
		occumap.theta(Idx+2) = midTheta + HalfInter;
	 	occumap.theta(Idx+3) = midTheta + HalfInter;
		occumap.rho(Idx) = 0;
		occumap.rho(Idx+1) = min(occumap.r(Order),maxr);
		occumap.rho(Idx+2) = min(occumap.r(Order),maxr);
		occumap.rho(Idx+3) = 0;
	end
	idx = find(occumap.theta<0);
	occumap.theta(idx) = occumap.theta(idx) + 2*pi;
	polar(occumap.theta,occumap.rho);
	[occumap.x,occumap.y] = pol2cart(occumap.theta,occumap.rho);
	patch(occumap.x,occumap.y,[0 0 1]);
	view(-90,90);
%	drawnow;
%end

%end
