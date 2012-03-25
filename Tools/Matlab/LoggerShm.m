function ret = LoggerShm(teamNumber, playerID)

if (nargin < 2)
  playerID = parse_hostname();
end
if (nargin < 1)
  teamNumber = 26;
end

playerID = 3;
teamNumber = 18;

global LOG

if isempty(LOG)
  LOG.camera = [];
end

% create shm interface
robot = shm_robot(teamNumber, playerID);

% flag of taking logs or not
LOG.get_log = 0;

% init window
figure(1);
clf;

% Log Button
hButton = uicontrol('Style','pushbutton','String','LOG',...
	'Position',[20 50 70 40],'Callback','LOG.get_log=1-LOG.get_log;');

while (1)
	rgb = robot.get_rgb();
	fig = image(rgb);
	drawnow;	
	
	if LOG.get_log == 1
	  ilog = length(LOG.camera) + 1;
	  LOG.camera(ilog).time = ilog;
	  LOG.camera(ilog).yuyv = robot.get_yuyv() + 0;
	  LOG.camera(ilog).headAngles = robot.vcmImage.get_headAngles() + 0; 
	  % TODO: store the IMU data
	  %LOG.camera(ilog).imuAngles = CAMERADATA.imuAngles;
	  LOG.camera(ilog).select = robot.vcmImage.get_select() + 0;
	
	  if (rem(ilog,5) == 0)
	    % print ticks to indicate that the logger is working
	    fprintf('.');
	  end
	
	  if (rem(ilog, 100) == 0)

	    savefile = ['./colortable/log_' datestr(now,30) '.mat'];
	    fprintf('\nSaving Log file: %s...', savefile)
	    save(savefile, 'LOG');
	    fprintf('done\n');
	
	    % clear log
	    LOG.camera = [];
	  end
	end

  pause(0.05);

end
