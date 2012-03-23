function MonitorShmSingle(team,player)

  max_player_id = 5; 
  % Should monitor run continuously?
  continuous = 1;
  draw_team = 0;
  tDisplay = .1; % Display every x seconds
  dInterval = 5; % Show vision in team view every x frames

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
    if shm_check(team2track,player2track)==0 
      disp('Team/Player ID error!');
      return;
    end
    robots{player2track,1}=shm_robot(team2track,player2track);
  end

  %% Enter loop
  figure(1);
  clf;
  if draw_team>0 
     set(gcf,'position',[1 1 900 900]);
  else
     set(gcf,'Position',[1 1 800 600])
  end


  tStart = tic;
  nUpdate = 0;
  %scale = 1; % 1: labelA, 4: labelB
  scale = 4; % 1: labelA, 4: labelB

  %% Initialize data
  t0=tic;
  t = toc( t0 );
  fprintf('Initialization time: %f\n',t);

  m=show_monitor();

  %% Update our plots
  while continuous
    nUpdate = nUpdate + 1;
    tStart = tic;
    if draw_team>0
      m.update_team( robots, scale, 1, player2track,mod(nUpdate,dInterval));
    else
      m.update( robots, scale, 1, player2track );
    end
    drawnow;
    tElapsed=toc(tStart);
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
