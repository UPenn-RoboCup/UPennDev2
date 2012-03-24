function MonitorShm(team,player)
%Usage: MonitorShm(1,2)       : single monitor
%       MonitorShm(1,[2 3 4]) : team monitor
%	MonitorShm(1)         : team auto-detect

  global MONITOR;

  max_player_id = 5; 
  % Should monitor run continuously?
  continuous = 1;
  draw_team = 0;
  tDisplay = .1; % Display every x seconds
  dInterval = 5; % Show vision in team view every x frames
  tDisplay = .2; % Display every x seconds

  if nargin==1
    %1 args.. track the whole players in the team
    team2track=team;
    robots = cell(max_player_id, 1);

    %Search SHM for players
    player2track=[];
    for i=1:max_player_id,
      if shm_check(team2track,i)>0 
        robots{i,1}=shm_robot(team2track,i);
	player2track=[player2track i];
      end
    end
    if length(player2track)==0
      disp('Team/Player ID error!');
      return;
    elseif length(player2track)>1
      draw_team=1;
    end
  else

    if nargin==2  %2 args... track specified player 
      team2track=team;player2track=player;
    else
      %Default value is the first field player (PLAYER_1_1)
      team2track = 1;player2track = 2;
    end

    if length(player2track)>1, draw_team=1; end
    for i=1:length(player2track)
      if shm_check(team2track,player2track(i))==0 
        disp('Team/Player ID error!');
        return;
      end
      robots{player2track(i),1}=shm_robot(team2track,player2track(i));
    end
  end

  %% Enter loop
  tStart = tic;
  nUpdate = 0;
  %scale = 1; % 1: labelA, 4: labelB
  scale = 4; % 1: labelA, 4: labelB

  %% Initialize data
  t0=tic;
  t = toc( t0 );
  fprintf('Initialization time: %f\n',t);

  MONITOR=show_monitor();
  MONITOR.init(draw_team);

  %% Update our plots
  while continuous
    nUpdate = nUpdate + 1;
    tElapsed=MONITOR.update( robots, scale, 1, player2track, draw_team, mod(nUpdate,dInterval));
    if(tElapsed<tDisplay)
      pause( tDisplay-tElapsed );
    end
  end

  %subfunction
  function h = shm_check(team, player)
    %Checks the existence of shm with team and player ID

    shm_dir='/dev/shm'; %For Linux... 

    shm_name_wcmRobot = sprintf('%s/wcmRobot%d%d%s', shm_dir, team, player, getenv('USER'));
    h=exist(shm_name_wcmRobot,'file');
  end

end
