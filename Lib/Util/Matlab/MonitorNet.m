% Robot to Track
RobotType = 'OP';  % 'Nao' or 'OP'
if (RobotType == 'OP')
	IP = '192.168.123.255';
elseif (RobotType == 'Nao')
  IP = '192.168.0.255';
end

% Players and team to track
nPlayers = 3;
teamNumbers = [18];
team2track = 1; % The order of teamNumber. 1st teamNumber or 2nd
player2track = 2; % PlayerID

% Should monitor run continuously?
continuous = 1;

%% Enter loop
figure(1);
clf;
tDisplay = .2; % Display every x seconds
tStart = tic;
nUpdate = 0;
scale = 4; % 1: labelA, 4: labelB

%% Initialize data
t0=tic;
robots = cell(nPlayers, length(teamNumbers));
for t = 1:length(teamNumbers)
    for p = 1:nPlayers
        robots{p,t} = net_robot(teamNumbers(t), p);
    end
end
t = toc( t0 );
fprintf('Initialization time: %f\n',t);

%% Update our plots
while continuous
    nUpdate = nUpdate + 1;
    
    %% Draw our information
    tElapsed=toc(tStart);
    if( tElapsed>tDisplay )
        tStart = tic;
%        % Show the monitor
%        show_monitor( robots, scale, team2track, player2track );
%        rgb = robots{player2track,team2track}.get_rgb_sub();
%				%rgb = robots{player2track,team2track}.get_rgb();
%        [yuv yuv_raw] = robots{player2track,team2track}.get_yuvSub();
%        drawnow;
    end
    
    %% Update our information
    if(monitorComm('getQueueSize',IP) > 0)
        msg = monitorComm('receive',IP);
%				disp([msg(1),tElapsed]);
        if ~isempty(msg)
            message = lua2mat(msg);
%            % Only track one robot...
            scale = robots{player2track,team2track}.update( message );
        end
    end
    
end
