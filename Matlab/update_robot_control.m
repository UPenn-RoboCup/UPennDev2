function [ nBytes ] = update_robot_control( msg )
	%UPDATE_ROBOT_CONTROL Send controller commands to the robot
	%   Just forwards the data for now
	
	global TELEOP_FD;
	%controller_cmd = msgpack('unpack', msg);
	nBytes = udp_send( TELEOP_FD, msg );

end

