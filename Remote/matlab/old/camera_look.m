function [ nBytes ] = camera_look( h_omap, ~, flags )
	%CAMERA_LOOK Tell the camera where to look
	% This is a GUI call
	
	global COMMAND_FD H_HCAMERA_AXES H_LCAMERA_AXES H_RCAMERA_AXES;
	
	if h_omap==H_HCAMERA_AXES 
		disp('center gaze')
	elseif h_omap==H_LCAMERA_AXES 
		disp('left gaze')
	elseif h_omap==H_RCAMERA_AXES 
		disp('right gaze')
	else
		return
	end
	
	%% Configuration variables
	h_fov = 60;
	v_fov = 60;
	cam_width = 320;
	cam_height = 240;
	cam_mid_x = cam_width/2;
	cam_mid_y = cam_height/2;
	focal_base = 320;
	focal_length = 260;
	
	gaze_data = {};
	gaze_data.cmd = 'unknown';
	if exist('flags')
		% Allow hcm directed gaze
	else
		% Add a waypoint
		point = get(gca,'CurrentPoint')
		x = point(1,1);
		y = cam_height-point(1,2);
		pan  = atan2( focal_length, x-cam_mid_x ) * 180/pi
		tilt = atan2( focal_length, y-cam_mid_y ) * 180/pi
		
		% Prep the command to send
		gaze_data.cmd = 'look';
		
		fprintf('Looking at %.2f, %.2f\n',x,y);
	end

	% Send the command
	 nBytes = 0;
end