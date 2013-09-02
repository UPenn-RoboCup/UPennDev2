function ret = models()
global MODELS LIDAR CONTROL DEBUGMON
MODELS.wheel_calc = @wheel_calc;
MODELS.tool_calc  = @tool_calc;
MODELS.door_calc  = @door_calc;

	%%%%%%%%%%%%
	%% Door calculations
	function data = door_calc(h,~,val)
		data=[];
	    points3d = LIDAR.selected_points;
	    LIDAR.clear_points();

	    disp('Door calculation...');
	    % Just get the handle first...
	    npoints = size(points3d,1);
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
		    disp('Grip is too far or too close!');
		    disp(grip_pos);
		    return;
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
        CONTROL.send_control_packet([],[],'hcm','door','handle',handle);
        % TODO: Draw another point on there, with the actual wheel center?
	end

	%%%%%%%%%%%%
	%% Tool calculations
	function data = tool_calc(h,~,val)
		data=[];
	    points3d = LIDAR.selected_points;
	    LIDAR.clear_points();

	    disp('Wheel calculation...');
	    if numel(points3d)<3*3
	    	return
	    end

		% NOTE: Assume a left right top clicking order!!
		leftrelpos  = points3d(size(points3d,1)-2, :);
		rightrelpos = points3d(size(points3d,1)-1, :);
		toprelpos   = points3d(size(points3d,1),   :);

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
        LIDAR.wheel_model = [handlepos handleyaw handlepitch handleradius];
        CONTROL.send_control_packet([],[],'hcm','wheel','model',LIDAR.wheel_model);
        % TODO: Draw another point on there, with the actual wheel center?
	end

		%%%%%%%%%%%%
	%% Wheel calculations
	function data = wheel_calc(h,~,val)
		data=[];
	    points3d = LIDAR.selected_points;
	    LIDAR.clear_points();

	    disp('Wheel calculation...');
	    if numel(points3d)<3*3
	    	return
	    end

		% NOTE: Assume a left right top clicking order!!
		leftrelpos  = points3d(size(points3d,1)-2, :);
		rightrelpos = points3d(size(points3d,1)-1, :);
		toprelpos   = points3d(size(points3d,1),   :);

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
        CONTROL.send_control_packet([],[],'hcm','wheel','model', wheel );
        % TODO: Draw another point on there, with the actual wheel center?
	end

ret = MODELS;
end