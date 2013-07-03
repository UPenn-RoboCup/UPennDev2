function [s_recv_data] = update_network( t_diff )
	%UPDATE_NETWORK Summary of this function goes here
	%   Detailed explanation goes here

	global H_NETWORK H_NETWORK_GRAPH RECENT_NETWORK_USAGE;
	
	% Grab the current rate
	net_usage_kb = RECENT_NETWORK_USAGE/1024;
	network_usage = sum( net_usage_kb, 2 );
	current_usage_rate = net_usage_kb(1) / t_diff
	
	% Print Network Usage and update FPS
	set( H_NETWORK, {'YDATA'}, num2cell( net_usage_kb, 1 )' );
	title( H_NETWORK, sprintf('Current rate %6.2f kB/s',current_usage_rate) );
	
	% Update recent history
	RECENT_NETWORK_USAGE = circshift(RECENT_NETWORK_USAGE,1);
	RECENT_NETWORK_USAGE(1,:) = 0;

end
