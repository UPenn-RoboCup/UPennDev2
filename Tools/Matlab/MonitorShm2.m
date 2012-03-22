%Minimal monitor for single robot in webots


team2track = 1;
player2track = 1;

% Should monitor run continuously?
continuous = 1;


%% Enter loop
figure(1);
clf;
tDisplay = .1; % Display every x seconds
tStart = tic;
nUpdate = 0;
%scale = 1; % 1: labelA, 4: labelB
scale = 4; % 1: labelA, 4: labelB

%% Initialize data
t0=tic;
robots = cell(player2track, 1);
robots{player2track,1}=shm_robot(team2track,player2track);
t = toc( t0 );
fprintf('Initialization time: %f\n',t);

%% Update our plots
while continuous
    nUpdate = nUpdate + 1;

    %% Draw our information
    tStart = tic;
    show_monitor2( robots, scale, team2track, player2track );
    drawnow;
    tElapsed=toc(tStart);

    if(tElapsed<tDisplay)
        pause( tDisplay-tElapsed );
    end

end
