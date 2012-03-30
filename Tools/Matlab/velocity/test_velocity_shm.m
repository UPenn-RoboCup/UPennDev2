%% Using the ramp webots test file
%% NOTE: mexBall is in Lib/Velocity
if( ~exist('robot','var') )
    startup;
    disp('Creating shm handle!');
    robot = shm_robot(99,2);
end

loop_fps = 48;
loop_twait = 1/loop_fps;
tLast = 0;
tFrame = 1/24;

%% Set up plot
figure(1);
clf;
h_q = quiver(0,0,0,0,'k','LineWidth',5,'MarkerSize',10);
hold on;
h_ball = plot(0,0,'mo','MarkerSize',16,'MarkerFaceColor','r');
xlim([-1 5]);
ylim([-5 5]);

%% Loop
while(1)
    loop_tstart=tic;
    m = robot.get_monitor_struct();
    ball = m.ball;
    
    tdiff = ball.t - tLast;
    if( tdiff==0 )
        continue;
    end
    nFrames = floor( tdiff / tFrame ) + 1;
    tLast = ball.t;
    filtered_ball = mexBall([ball.x,ball.y],nFrames); %[x y vx vy ep evp]
    [theta rho] = cart2pol(filtered_ball(3),filtered_ball(4));
    rho = 100*rho;
    set( h_q, 'UData', rho*cos(theta), 'VData', rho*sin(theta),...
        'XData',filtered_ball(1),'YData',filtered_ball(2));
    set( h_ball, 'XData',filtered_ball(1),'YData',filtered_ball(2) );
    drawnow;
    
    loop_tf = toc(loop_tstart);
    pause( max(loop_twait-loop_tf,0) );
end