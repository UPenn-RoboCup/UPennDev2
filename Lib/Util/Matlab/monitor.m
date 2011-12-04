function ret = monitor(adflag, teamNumber, playerID)
% adflag - 0 or 1 : Enable Advanced monitor
% teamNumber : 
% playerID :

% monitor mode depends on input
if (nargin == 0)
	% No input enable simple monitor
	adflag = 0;
	% No input enable network monitor
	webots = 0;
elseif (nargin > 1)
	webots = 1;
end


% create shm interface 
if (webots == 1)
	robot = shm_robot(teamNumber,playerID);
end

%Ashleigh
%parse array and monitor packets for each robot
yuyv = construct_array('yuyv');
labelA = construct_array('labelA');
labelB = construct_array('labelB');
cont = 1;
arr = [];

%% Establish our variables to display
y = [];
a = [];
b = [];
ballB = [];

cbk=[0 0 0];cr=[1 0 0];cg=[0 1 0];cb=[0 0 1];cy=[1 1 0];cw=[1 1 1];
cmap=[cbk;cr;cy;cy;cb;cb;cb;cb;cg;cg;cg;cg;cg;cg;cg;cg;cw];

% Window Arrange for Advanced Monitor
if (adflag == 1)
	% Monitor Size for Advanced Monitor
	scrz = get(0,'ScreenSize');
	% Monitor Ratio 2:1
	fSizeX = scrz(3)/6*3;
	fSizeY = scrz(3)/12*3;
	figure('Pos',[scrz(3)/2-fSizeX/2 scrz(4)/2-fSizeY/2 fSizeX fSizeY]);
	% advanced monitor 2x3 subplots
	nCol = 3;
else
	figure(1);
	% simple monitor 2x2 subplots
	nCol = 2;
end


tDisplay = .1; % Display every .1 seconds
tStart = tic;
while cont
	if (webots == 1)
		% Record information from shm
    	y = robot.get_yuyv()';
		a = robot.get_labelA()';
		b = robot.get_labelB()';
		wlabelA = robot.vcmImage.get_width()/2;
		hlabelA = robot.vcmImage.get_height()/2;
		wlabelB = wlabelA/4;
		hlabelB = hlabelA/4;
		% Circle the ball in the images from shm
		scale = 4; % For labelB
		ball = [];
		ballB = []; 
		if (robot.vcmBall.get_detect() == 1)
			centroidA = {};
			centroidValue = robot.vcmBall.get_centroid();
			centroidA.x = centroidValue(1);
			centroidA.y = centroidValue(2);
			radius = robot.vcmBall.get_axisMajor()/2;
			ball = [centroidA.x-radius centroidA.y-radius ...
					2*radius 2*radius];
			centroidB = {};
			centroidB.x = centroidValue(1)/scale;
			centroidB.y = centroidValue(2)/scale;
			radiusB = robot.vcmBall.get_axisMajor()/2/scale;
            ballB = [centroidB.x-radiusB centroidB.y-radiusB ...
						2*radiusB 2*radiusB];
		end
		% show goal
		goal = {};
		if (robot.vcmGoal.get_detect() == 1)
			goal.type = robot.vcmGoal.get_type();
			goal.color = robot.vcmGoal.get_color();
			gBB1 = scale*robot.vcmGoal.get_postBoundingBox1();
			gBB2 = scale*robot.vcmGoal.get_postBoundingBox2();
			goal.bbox1 = [gBB1(2),gBB1(1),gBB1(2)-gBB1(4)+1,gBB1(1)-gBB1(3)+1];
			goal.bbox2 = [gBB2(2),gBB2(1),gBB2(2)-gBB2(4)+1,gBB2(1)-gBB2(3)+1];
			goal.v1 = robot.vcmGoal.get_v1();
			goal.v2 = robot.vcmGoal.get_v2();	
		end
		% Show freespace range
		freeA = {};
		if (robot.vcmFreespace.get_detect() == 1)
			freeColA = robot.vcmFreespace.get_nCol();
			freeValueA = robot.vcmFreespace.get_boundA();
			freeA.x = freeValueA(1:numel(freeValueA)/2);
			freeA.y = freeValueA(numel(freeValueA)/2+1:...
											numel(freeValueA));
			TurnPnt = robot.vcmFreespace.get_turn();
			posTurnPntUp = find(TurnPnt == 1);
			posTurnPntDn = find(TurnPnt == -1);
            freeA.xTurnUp = freeValueA(posTurnPntUp);
			freeA.yTurnUp = freeValueA(posTurnPntUp + freeColA);
			freeA.xTurnDn = freeValueA(posTurnPntDn);
			freeA.yTurnDn = freeValueA(posTurnPntDn + freeColA);
		%{
			freeARegAlpha = robot.vcmFreespace.get_a();
			freeARegBeta = robot.vcmFreespace.get_b();
			freeA.xReg = 1 : wlabelA;
			freeA.yReg = freeARegAlpha + freeARegBeta * freeA.xReg; 
		%}
		end
		% show boundary in robot coordinate
		bd = {};
		if (robot.vcmBoundary.get_detect() == 1)
			bdTop = robot.vcmBoundary.get_top();
			bdBtm = robot.vcmBoundary.get_bottom();
			bdCol = size(bdTop,2)/2;
			bd.topy = bdTop(1,1:bdCol);
			bd.topx = bdTop(1,bdCol+1:2*bdCol);
			bd.btmy = bdBtm(1,1:bdCol);
			bd.btmx = bdBtm(1,bdCol+1:2*bdCol);
		end
	else
	    %% Record our information
    	if(monitorComm('getQueueSize') > 0)
        	msg = monitorComm('receive');
        	if ~isempty(msg)
            	msg = lua2mat(char(msg))
            	if (isfield(msg, 'arr'))
                	y = yuyv.update(msg.arr);
                	a = labelA.update(msg.arr);
                	b = labelB.update(msg.arr);
				% Circle the ball in the images
	            elseif( isfield(msg, 'ball') ) 
    	            scale = 4; % For labelB
        	        ball = msg.ball;
            	    ballB = [];
                	if(ball.detect==1)                    
	                    centroidB = msg.ball.centroid;
    	                centroidB.x = centroidB.x/scale;
        	            centroidB.y = centroidB.y/scale;
            	        radiusB = (msg.ball.axisMajor/scale)/2;
                	    ballB = [centroidB(1)-radiusB centroidB(2)-...
								radiusB 2*radiusB 2*radiusB];
	                end
    	        end
        	end
	    end
	end
    
    
    %% Draw our information
    tElapsed=toc(tStart);
    if( tElapsed>tDisplay )
        %disp(tElapsed)
        tStart = tic;
        
        if ~isempty(y)
            subplot(2,nCol,1);
            imagesc(yuyv2rgb(y'));
            title('Received image.')
        end
        if ~isempty(a)
            subplot(2,nCol,2);
            imagesc(a);
            colormap(cmap);
            title('Received Label A.');
        end
        if ~isempty(ball)
            subplot(2,nCol,2);
            hold on;
            plot(centroidA.x, centroidA.y,'w+')
            rectangle('Position', ball, 'Curvature',[1,1])
            hold off;
            %disp('Plotting ball');
        end
        if ~isempty(b)
            subplot(2,nCol,3);
            image(b);
            colormap(cmap);
            title('Received Label B.');
        end

        if ~isempty(ballB)
            subplot(2,nCol,3);
            hold on;
            plot(centroidB.x, centroidB.y,'k+')
            rectangle('Position', ballB, 'Curvature',[1,1])
            hold off;
            %disp('Plotting ball');
        end
		
		% Enable Advanced Display
		if (adflag == 1)
			% show goal 
			if ~isempty(goal)
				subplot(2,nCol,2);
				if (goal.color == 2) % yellow goal
					bbColor = 'Blue';
				elseif (goal.color == 4)
					bbColor = 'Yellow';
				end
				hold on;
%				disp(goal.bbox1);
%				disp(goal.bbox2);
				if (numel(goal.bbox1==0)==0) % post 1 available
%					disp('Draw post1');
					rectangle('Position',goal.bbox1,'Curvature',[0,0],...
								'EdgeColor',bbColor,'LineWidth',2);
				end
				if (numel(goal.bbox2==0)==0) % post 2 available
%					disp('Draw post2');
					rectangle('Position',goal.bbox2,'Curvature',[0,0],...
								'EdgeColor',bbColor,'LineWidth',2);					
				end
				hold off;
			end
			% Show freespace boundary
			if ~isempty(freeA)
				subplot(2,nCol,2);
				hold on;
			%{
			    %Show freespace as bars
			    for colIdx = 1 : freeColA
					plot([freeA.x(colIdx),freeA.x(colIdx)],[hlabelA,...
							freeA.y(colIdx)],'b--','LineWidth',1);
				end
			%}

				% Show freespace as image boundary
				plot(freeA.x,freeA.y,'m--','LineWidth',2);
				plot(freeA.xTurnUp,freeA.yTurnUp,'^r');
				plot(freeA.xTurnDn,freeA.yTurnDn,'vr');	
				%plot(freeA.xReg,freeA.yReg,'w--','LineWidth',2);
				hold off;
			end
	        if ~isempty(a)
    	        subplot(2,nCol,4);
				plot_field();
	            title('World Coordinate');
    	    end
        	if ~isempty(y)
            	subplot(2,nCol,5);
				%plot_coordinate();
    	        title('Robot Coordinate');
        	end
			% show visible boundary in robot coordinate
			if ~isempty(bd)
				subplot(2,nCol,5);
				% show top boundary
				plot(bd.topx,bd.topy,'--');
				hold on;
				% show bottom boundary
				plot(bd.btmx,bd.btmy,'--');
				% close boundary
				plot([bd.topx(1),bd.btmx(1)],[bd.topy(1),bd.btmy(1)],'--');
				plot([bd.topx(bdCol),bd.btmx(bdCol)],[bd.topy(bdCol),bd.btmy(bdCol)],'--');
				axis([-1.5 1.5 0 3]);
				set(gca,'Xdir','reverse');
				hold off;
			end
        	if ~isempty(b)
            	subplot(2,nCol,6);
				plot_occmap(robot);
        	    title('Occupancy Map');
	        end
		end
        drawnow;
    end
    
end


