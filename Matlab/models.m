function ret = models()
global MODELS LIDAR CONTROL DEBUGMON
MODELS.wheel_calc = @wheel_calc;
MODELS.tool_calc  = @tool_calc;
MODELS.door_calc  = @door_calc;
MODELS.step_calc  = @step_calc;
MODELS.ooi = '';
% TODO: Each model has properties
MODELS.door  = [];
MODELS.wheel = [];
MODELS.waypoints = [];

	%%%%%%%%%%%%
	%% Door calculations
	function data = door_calc(h,~,val)
		data=[];
	    points3d = LIDAR.selected_points;
	    LIDAR.clear_points();

	    MODELS.ooi = 'door'

	    disp('Door calculation...');
	    % Just get the handle first...
	    npoints = size(points3d,1)

	    % If no points, then just set this as the object of interest
		if npoints == 0
			%CONTROL.send_control_packet('GameFSM',MODELS.ooi);
			return
		end

		% We just want two point for the handle
		% TODO: add the hinge
	    if npoints ~= 2
	    	return;
	    end

		% NOTE: Assume a left right top clicking order!!
		axis_pos = points3d(1, :);
		end_pos  = points3d(2, :);
		axis_to_end = end_pos - axis_pos;

		disp(axis_pos);
		disp(end_pos);

		% Find the center of the wheel
		grip_pos = (axis_pos+end_pos) / 2;
		if grip_pos(1) > .5 || grip_pos(1) < 0.10
		    % x distance in meters
		    disp('WARNING: Grip is too far or too close!');
		    disp(grip_pos);
		    %return;
		end

		% Find the radius of the wheel
		handle_length = norm(axis_to_end);
		handle_length = min(handle_length,1);
		if handle_length<0.05
		    % 5cm length minimum
		    disp('Handle length is too small!');
		    disp(handle_length);
		    return;
		end

        handle_yaw = atan2(...
            axis_to_end(2), ...
            axis_to_end(1));
        % TODO: yaw checks

        % TODO: roll
        handle_roll = 0;
        
        % Debug message
        wheel_str = sprintf(...
            'Pos %.2f %.2f %.2f\nY %.1f R %.1f Len %.2f',...
            grip_pos(1),grip_pos(2),grip_pos(3),...
            handle_yaw*180/pi, handle_roll*180/pi, handle_length );
        DEBUGMON.addtext(wheel_str);
        
        % Overwrite wheel estimate?
        % TODO: use two separate wheel estimates?
        % TODO: send this data to the robot
        handle = [grip_pos handle_yaw handle_roll handle_length];
        CONTROL.send_control_packet('GameFSM',MODELS.ooi,'hcm','door','handle',handle);
        % TODO: Draw another point on there, with the actual wheel center?

	end

	%%%%%%%%%%%%
	%% Tool calculations
	function data = tool_calc(h,~,val)	
		data=[];
	    points3d = LIDAR.selected_points;
	    LIDAR.clear_points();
	    MODELS.ooi = 'tool';
	    npoints = size(points3d,1);

	    % If no points, then just set this as the object of interest
		if npoints == 0 
			return
		end

	    disp('Tool calculation...');
	    % Need at least three points to determine the circle
	    if npoints<2
	    	return
	    end

		% top and bottom clicking order
		
		toprelpos = points3d(npoints-1, :);
		bottomrelpos   = points3d(npoints,   :);

		% Find the center of the wheel
		grippos = (toprelpos+bottomrelpos)/2;
		        
        grippitch = atan2( ...
        	toprelpos(1)-bottomrelpos(1),...
            toprelpos(3)-bottomrelpos(3));
        
        % Debug message
        tool_str = sprintf(...
            'Pos %.2f %.2f %.2f\nP %.1f',...
            grippos(1),grippos(2),grippos(3),...
            grippitch*180/pi);
        DEBUGMON.addtext(tool_str);
                
        tool = [grippos grippitch];        
        CONTROL.send_control_packet('GameFSM',MODELS.ooi,'hcm','tool','model', tool );
    end

		%%%%%%%%%%%%
	%% Wheel calculations
	function data = wheel_calc(h,~,val)
		data=[];
	    points3d = LIDAR.selected_points;


	    LIDAR.clear_points();

	    MODELS.ooi = 'wheel';

	    npoints = size(points3d,1);

	    % If no points, then just set this as the object of interest
		if npoints == 0
			return
		end

	    disp('Wheel calculation...');
	    % Need at least three points to determine the circle
	    if npoints<3
	    	return
	    end

		% NOTE: Assume a left right top clicking order!!
		leftrelpos  = points3d(npoints-2, :);
		rightrelpos = points3d(npoints-1, :);
		toprelpos   = points3d(npoints,   :);

		% Find the center of the wheel
		handlepos = (leftrelpos+rightrelpos) / 2;

		if handlepos(1) > 1 || handlepos(1) < 0.10
		    % x distance in meters
		    disp('Handle is too far or too close!');
		    disp(handlepos);
		    return;
		end

		% Find the radius of the wheel
		handleradius = norm(leftrelpos-rightrelpos)/2;
		%hack to fix for valve grip radius
		handleradius = handleradius + 0.02; 

		if handleradius>1 || handleradius<0.10
		    % radius in meters
		    disp('Radius is too big or too small!');
		    disp(handleradius);
		    return;
		end

        handleyaw = atan2(...
            leftrelpos(2)-rightrelpos(2), ...
            leftrelpos(1)-rightrelpos(1)) ...
            - pi/2;
        % TODO: yaw checks

        handlepitch = atan2( ...
        	toprelpos(1)-handlepos(1),...
            toprelpos(3)-handlepos(3) );
        % TODO: pitch checks

        % Debug message
        wheel_str = sprintf(...
            'Pos %.2f %.2f %.2f\nY %.1f P %.1f Rad %.2f',...
            handlepos(1),handlepos(2),handlepos(3),...
            handleyaw*180/pi,handlepitch*180/pi,handleradius );
        DEBUGMON.addtext(wheel_str);
        
        % Overwrite wheel estimate?
        % TODO: use two separate wheel estimates?
        % TODO: send this data to the robot
        wheel = [handlepos handleyaw handlepitch handleradius];
        CONTROL.send_control_packet('GameFSM',MODELS.ooi,'hcm','wheel','model', wheel );
        % TODO: Draw another point on there, with the actual wheel center?

    end

    %% Step Calculations
    function data = step_calc(h,~)
      data=[];
	  points3d = LIDAR.selected_points;
	  LIDAR.clear_points();

	  MODELS.ooi = 'step'
	  disp('Step calculation...');
	    
      % Get the step first
	  npoints = size(points3d,1);

	  % If no points, then just set this as the object of interest
	  if npoints == 0
          %CONTROL.send_control_packet('GameFSM',MODELS.ooi);
		  return
	  end

      % We just do one step each time for now
      % TODO: three points for foot pitch and roll
        if npoints ~= 1
          return;
        end

      footpos = points3d(1,:);
      footpos_global = LIDAR.transform_global(footpos);
      disp(footpos);

      % Check the distance
      % TODO: tune the threshold
      if footpos(1) > .25 || footpos(1) < 0.10
          % x distance in meters
          disp('WARNING: Block is too far or too close!');
          disp(footpos);
          return;
      end
      
      % Check the height
      % TODO: tune the threshold
      % TODO: ground height is quite noisy. Should be in LIDAR or SLAM
      ground_height = -0.75;
      step_height = footpos_global(3) - ground_height;
      if step_height > .2 || step_height< -.05
          % x distance in meters
          disp('WARNING: Block is too high! Or it is a pit!');
          disp(step_height);
          return;
      end
      
      % Check the y position
      % TODO: check which foot to use (currently assueme RIGHT)
      if footpos(2) > 0.2+0.2 || footpos(2)<0.2
          disp('WARNING: Improper foothold!');
          disp(footpos);
          return;
      end

        % TODO: send this data to the robot
%         GAMEFSM or MOTIONFSM 
 
    end

ret = MODELS;
end
