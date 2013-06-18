function [ nBytes ] = update_omap( omap_fd )

	%UPDATE_OMAP Update the omap plot
		% Return the number of network bytes
		
	global H_SLAM;
    
    
	nBytes = 0;
	while udp_recv('getQueueSize',omap_fd) > 0
		udp_data = udp_recv('receive',omap_fd);
		nBytes = nBytes+numel(udp_data);
    end
    
    
    
	thor_omap = djpeg(udp_data);
    
    
    
	set(H_SLAM,'Cdata', thor_omap);
	% Return the number of network bytes
	nBytes = numel(udp_data);
end
