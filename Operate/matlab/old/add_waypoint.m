function [ nBytes ] = add_waypoint( h_omap, ~, flags )
	%ADD_WAYPOINT Add a waypoint to the occupancy map and command the robot
	% This is a GUI call
	
	global WAYPOINTS 
  global SLAM

  H_WAYPOINTS = SLAM.wayline;


	waypoints_data = {};
	waypoints_data.cmd = 'unknown';
	if exist('flags')
		% Clear waypoints
		WAYPOINTS = [];
		set( H_WAYPOINTS, 'XData', [] );
		set( H_WAYPOINTS, 'YData', [] );
		% Prep the command to send
		waypoints_data.cmd = 'clear';
	else
		% Add a waypoint
		point = get(gca,'CurrentPoint');
		x = point(1,1);
		y = point(1,2);
		WAYPOINTS = [WAYPOINTS;[x,y]];
		set( H_WAYPOINTS, 'XData', WAYPOINTS(:,1) );
		set( H_WAYPOINTS, 'YData', WAYPOINTS(:,2) );
		% Prep the command to send
		waypoints_data.cmd = 'add';
		waypoints_data.waypoints = WAYPOINTS;
		fprintf('Adding waypoint %.2f, %.2f\n',x,y);
	end

	% Send the command
	waypoints_msg = msgpack('pack', waypoints_data);
	nBytes = udp_send( COMMAND_FD, waypoints_msg );
	fprintf('Sent! %d bytes\n',nBytes);
end
